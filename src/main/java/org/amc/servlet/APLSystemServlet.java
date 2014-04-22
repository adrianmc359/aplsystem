package org.amc.servlet;

import java.io.IOException;
import java.sql.SQLException;
import java.util.List;

import javax.servlet.RequestDispatcher;
import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;

import org.amc.servlet.action.PartActionFactory;
import org.amc.servlet.action.SavePartAction;
import org.amc.servlet.action.SearchPartAction;
import org.amc.servlet.model.Part;
import org.amc.servlet.model.PartForm;
import org.amc.servlet.validator.Part_Validator;
import org.apache.log4j.Logger;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.ApplicationContext;
/**
 * Servlet implementation class APLSystemServlet
 */
@WebServlet(
		description = "Dispatching Servlet for the Problem Database", 
		urlPatterns = { 
				"/APLSystemServlet", 
				"/Part_display",
				"/Part_search",
				"/Part_save", 
				"/Problem_save", 
				"/Problem_display", 
				"/ProblemDescription_save", 
				"/ProblemDescription_display", 
				"/SearchProblemDatabase",
				"/logout"
		},loadOnStartup=1)

public class APLSystemServlet extends HttpServlet 
{
	private static final long serialVersionUID = 334034039L;

	private PartActionFactory partActionFactory;
	
	private static Logger logger=Logger.getLogger(APLSystemServlet.class);

	/**
	 * @see HttpServlet#doGet(HttpServletRequest request, HttpServletResponse response)
	 */
	public void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException 
	{
		process(request, response);
	}

	/**
	 * @see HttpServlet#doPost(HttpServletRequest request, HttpServletResponse response)
	 */
	protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException 
	{
		process(request, response);
	}
	
	/**
	 * Handles incoming requests
	 */
	
	private void process (HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException 
	{
		
		String referal=request.getRequestURI();
		logger.debug(referal);
		
		//Handle Part Page
		if(referal.endsWith("Part_save"))
		{
			
			saveJobTemplate(request, response);
		}
		else if(referal.endsWith("Part_display"))
		{
			displayJobTemplate(request, response);
		}
		else if(referal.endsWith("Part_search"))
		{
			searchJobTemplate(request, response);
		}// To log out the current user
		else if(referal.endsWith("logout"))
		{
			logout(request,response);
		}
		else if(referal.endsWith("APLSystemServlet"))
		{
//			PrintWriter writer=response.getWriter();
//			response.setContentType("text/html");
//			
//			writer.println("<!doctype html>"
//					+ "<HTML>"
//					+ "<BODY>"
//					+ "<a href='JobTemplate_display'>JobTemplate</a>"
//					+ "</BODY>"
//					+ "</HTML>");
//			writer.flush();
			RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/Main.jsp");
			rd.forward(request, response);
		}
		
	}
	
	/**
	 * Saves the Part to the Database
	 * @param request
	 * @param response
	 * @throws ServletException
	 * @throws IOException
	 */
	private void saveJobTemplate(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		
		//check if page is in create or edit mode
		String mode=request.getParameter("mode");
		logger.debug(String.format("mode:[%s]", mode));//debug
		//create form
		PartForm jForm=new PartForm();
		
		//If an ID parameter is passed add it to the form 
		String id=request.getParameter("id");
		if(id!=null)
		{
			jForm.setId(id);
		}
		jForm.setName(request.getParameter("name"));
		jForm.setColour(request.getParameter("colour"));
		//checkedbox
		String external=request.getParameter("external");
		if(external==null)
		{
			jForm.setExternal(false);
		}
		else
		{
			jForm.setExternal(true);
		}
		
		jForm.setPart_id(request.getParameter("part_id"));
		jForm.setQss_no(request.getParameter("qss_no"));
		jForm.setRevision(request.getParameter("revision"));
		jForm.setVersion(request.getParameter("version"));
		jForm.setCompany(request.getParameter("company"));
		
		//Validate Form
		Part_Validator validator=new Part_Validator();
		List<String> errors=validator.validate(jForm);
		
		//Check if user is a role to allow changes to the database
		if(!(request.isUserInRole("qc")||(request.isUserInRole("manager"))))
		{
			errors.add("User has no permissions to alter Part definitions!");
		}
				
		//If form validates with no errorts
		if(errors.isEmpty())
		{
		
			//create model
			Part job=new Part();
			job.setCompany(jForm.getCompany());
			job.setName(jForm.getName());
			job.setColour(jForm.getColour());
			job.setExternal(jForm.getExternal());
			job.setQss_no(jForm.getQss_no());
			job.setRevision(jForm.getRevision());
			job.setVersion(jForm.getVersion());
			job.setPart_id(jForm.getPart_id());
			
			String dispatcherURL="";
			SavePartAction action=partActionFactory.getSaveJobTemplateAction();
			try
			{
				// New Part to Database
				if(mode==null||mode.equals("Enter"))
				{
					action.save(job);
					request.setAttribute("form",jForm);
					request.setAttribute("result", job.toString()+" saved");
					dispatcherURL="/WEB-INF/JSP/Part.jsp";
				}
				else if(mode.equals("Edit"))
				{
					//Current Part is updated in the Database
					job.setId(Integer.parseInt(jForm.getId()));
					action.edit(job);
					dispatcherURL="Part_search";
					response.sendRedirect(request.getContextPath()+"/Part_search"); // Goto the Search Window
					return; // Exit function 
				}
				else
				{
					throw new ServletException("Form received can't be processed");
				}

				//request.removeAttribute("form");
				request.removeAttribute("errors");
				RequestDispatcher rd=request.getRequestDispatcher(dispatcherURL);
				
				rd.forward(request, response);
			}
			catch(NumberFormatException se)
			{
				throw new ServletException(se);
			}
			catch(SQLException se)
			{
				
				throw new ServletException("Database not available:"+se.getMessage());
			}
			
			
		}
		else
		{
			//if the form doesn't validate without errors then
			request.setAttribute("errors", errors);
			request.setAttribute("form", jForm);			
			
			RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/Part.jsp");
			rd.forward(request, response);
		}
	}
	
	/**
	 * display a blank Part page
	 * @param request
	 * @param response
	 * @throws ServletException
	 * @throws IOException
	 */
	private void displayJobTemplate(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/Part.jsp");
		rd.forward(request, response);
	}
	
	/**
	 * Create SearchPage
	 * @param request
	 * @param response
	 * @throws ServletException
	 * @throws IOException
	 */
	private void searchJobTemplate(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		
		//check to if in search or edit mode TODO add delete mode
		String mode=request.getParameter("mode");
		//Passed to use as a search term
		String searchWord=request.getParameter("search");
		//Passed if an entry is to be edited
		String idValue=request.getParameter("edit");
		
		//Debug
		logger.debug(String.format("mode:[%s] searchWord:[%s] ID:[%s]%n", mode,searchWord,idValue));
		
		
		
		//create an action
		SearchPartAction sjt=this.partActionFactory.getSearchJobTemplateAction();
		String dispatchURL=null;
		try
		{
			//if the page is to do a search
			if(mode==null || mode.equals("search"))
			{
				List<Part> list=null;
				//To check to search for all entries or entries where name=searchWord
				if(searchWord==null||searchWord.equals(""))// search for all entries
				{
					list=sjt.search();
				}
				else //search for entry where name=searchWord 
				{
					
					list=sjt.search("name",searchWord);
				}
				request.setAttribute("parts", list); //Add the result list to the request object to be used by the JSP page
				
				//debug
				logger.debug(String.format("%d results returned %n",list.size()));
			
				dispatchURL="/WEB-INF/JSP/PartsSearchPage.jsp";
				
			}
//			else if(mode!=null && mode.equals("edit") && idValue!=null)// Edit mode
//			{
//				//open the Part JSPage in edit mode
//				Part job=sjt.getPart(idValue);
//				dispatchURL="/JSP/Part.jsp";
//				request.setAttribute("form", job);
//				request.setAttribute("mode","edit");
//				
//			}
//			else if(mode!=null && mode.equals("add"))
//			{
//				//open the Part JSPage in add mode
//				dispatchURL="/JSP/Part.jsp";
//			}
			else if(mode!=null)
			{
				if(mode.equals("add")||idValue==null) //idValue will equal null if the checked box isn't selected
				{
					//open the Part JSPage in add mode
					dispatchURL="/WEB-INF/JSP/Part.jsp";
				}
				else if(mode.equals("edit")&&idValue!=null)
				{
					//open the Part JSPage in edit mode
					Part job=sjt.getPart(idValue);
					dispatchURL="/WEB-INF/JSP/Part.jsp";
					request.setAttribute("form", job);
					request.setAttribute("mode","edit");
				}
			}
			RequestDispatcher rd=request.getRequestDispatcher(dispatchURL);
			rd.forward(request, response);
		}
		catch(SQLException se)
		{
			
			se.printStackTrace();
			throw new ServletException("Database not available:"+se.getMessage());
		}
	}
	
	private void logout(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		response.setHeader("Cache-Control", "no-cache");
		response.setHeader("Pragma", "no-cache");
		HttpSession session=request.getSession(false);
		if(session !=null)
		{
			session.invalidate();
		}
		request.logout();
		
		//request.authenticate(response);
		response.sendRedirect(getServletConfig().getServletContext().getContextPath());
	}
	
	@Autowired
	public void setJobActionFactory(PartActionFactory aJobActionFactory)
	{
		this.partActionFactory=aJobActionFactory;
	}
	

	@Override
	public void init() throws ServletException
	{
//		ServletContext sc=getServletContext();
//		Enumeration<String> list=sc.getInitParameterNames();
//		System.out.println("----------INIT---------");
//		while(list.hasMoreElements())
//		{
//			System.out.println(list.nextElement());
//		}
//		Enumeration<String> list2=sc.getAttributeNames();
//		System.out.println("----------Attibutes---------");
//		while(list2.hasMoreElements())
//		{
//			System.out.println(list2.nextElement());
//		}
		ApplicationContext context2=(ApplicationContext)getServletContext().getAttribute("org.springframework.web.context.WebApplicationContext.ROOT");
		//ApplicationContext context= new ClassPathXmlApplicationContext("PartsContext.xml");
		setJobActionFactory((PartActionFactory)context2.getBean("partActionFactory"));
		
		super.init();
	}

	
	
}

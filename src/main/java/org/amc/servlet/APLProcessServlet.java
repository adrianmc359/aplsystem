package org.amc.servlet;

import java.io.IOException;
import java.sql.SQLException;
import java.util.List;
import java.util.Map;

import javax.servlet.RequestDispatcher;
import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.amc.servlet.action.MaterialActionFactory;
import org.amc.servlet.action.ProcessActionFactory;
import org.amc.servlet.action.SaveProcessSheetAction;
import org.amc.servlet.action.SearchProcessSheetAction;
import org.amc.servlet.model.Material;
import org.amc.servlet.model.MouldingProcess;
import org.amc.servlet.model.MouldingProcessForm;
import org.amc.servlet.validator.ProcessForm_Validator;
import org.apache.log4j.Logger;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.context.WebApplicationContext;

/**
 * Servlet implementation class APLProcessServlet
 */
@WebServlet(description = "To serve up Process Sheet Data", 
			urlPatterns = 
			{
				"/APLProcessServlet",
				"/ProcessSheet_display",
				"/ProcessSheet_search",
				"/ProcessSheet_save", 
				"/ProcessSheet_analysis"
			},loadOnStartup=2)


public class APLProcessServlet extends HttpServlet 
{
	private static final long serialVersionUID = 1L;
	
	private static Logger logger=Logger.getLogger(APLProcessServlet.class);
       
	private ProcessActionFactory processActionFactory;
	
	private MaterialActionFactory materialActionFactory;

	/**
	 * @see HttpServlet#doGet(HttpServletRequest request, HttpServletResponse response)
	 */
	protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException 
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
	
	private void process(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		String referal=request.getRequestURI();
		logger.debug(referal);
		
		
		//Handle JobTemplate Page
		if(referal.endsWith("ProcessSheet_save"))
		{
			saveProcessSheet(request, response);
		}
		else if(referal.endsWith("ProcessSheet_display"))
		{
			displayProcessSheet(request, response);
		}
		else if(referal.endsWith("ProcessSheet_search"))
		{
			searchProcessSheets(request, response);
		}
		else if(referal.endsWith("APLProcessServlet"))
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

	private void saveProcessSheet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		logger.debug(String.format("Context Path:"+request.getContextPath()));
		//check if page is in create or edit mode
		String mode=request.getParameter("mode");
		logger.debug(String.format("SaveProcessSheet:mode:[%s]%n", mode));//debug
		logger.debug(String.format("SaveProcessSheet:"+request.getParameter("dateOfIssue")));
		//create form
		
		//If an ID parameter is passed add it to the form
		MouldingProcessForm jForm=(MouldingProcessForm)request.getAttribute("processSheet");
		logger.debug(String.format("\nSubmitted Process:"+jForm));
//		String id=request.getParameter("id");
//		if(id!=null)
//		{
//			jForm.setId(id);
//		}

		
		//Validate Form
		ProcessForm_Validator validator=new ProcessForm_Validator();
		List<String> errors=validator.validate(jForm);
		
		//Check if user is a role to allow changes to the database
		if(!(request.isUserInRole("qc")||(request.isUserInRole("manager"))))
		{
				errors.add("User has no permissions to alter Process sheet definitions!");
		}
		
		//If form validates with no errors
		if(errors.isEmpty())
		{
		
			//create model
			MouldingProcess processSheet;
			
			//Default dispatch URL
			//String dispatcherURL="/ProcessSheet_search";
			
			SaveProcessSheetAction action=processActionFactory.getSaveProcessSheetAction();
			try
			{
				processSheet=MouldingProcessForm.getMouldingProcess(jForm);
				// New JobTemplate to Database
				if(mode==null||mode.equals("Enter"))
				{
					logger.debug(String.format("SaveProcessSheet:Entering entry into database"));
					action.save(processSheet);
					response.sendRedirect(request.getContextPath()+"/ProcessSheet_search"); // Goto the Search Window
					return; // Exit function 
				}
				else if(mode.equals("edit"))
				{
					logger.debug(String.format("SaveProcessSheet:Editing entry into database"));
					//Current JobTemplate is updated in the Database
					//processSheet.setId(Integer.parseInt(jForm.getId()));
					processSheet.setId(Integer.parseInt(jForm.getId()));
					action.edit(processSheet);
					response.sendRedirect(request.getContextPath()+"/ProcessSheet_search"); // Goto the Search Window
					return; // Exit function 
				}
				else
				{
					throw new ServletException("Form received can't be processed");
				}

				//request.removeAttribute("form");
				//request.removeAttribute("errors");
				//RequestDispatcher rd=request.getRequestDispatcher(dispatcherURL);
				
				//rd.forward(request, response);
			}
			catch(NumberFormatException se)
			{
				throw new ServletException(se);
			}
			catch(SQLException se)
			{
				
				throw new ServletException(se);
			}
			catch(Exception e)
			{
				throw new ServletException(e);
			}
			
			
		}
		else
		{
			try
			{
			//if the form doesn't validate without errors then
			//Remember page is in edit mode
				if(mode.equals("edit")) 
				{
					request.setAttribute("mode", mode);
				}
			//Get List of Material
				Map<Integer,Material> materials=materialActionFactory.getSearchMaterialAction().search();

				request.setAttribute("materials", materials);
				request.setAttribute("errors", errors);
				request.setAttribute("form", jForm);			
				
				RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/ProcessPage.jsp");
				rd.forward(request, response);
			}
			catch(SQLException sqle)
			{
				throw new ServletException(sqle);
			}
		}
	}
	
	private void displayProcessSheet(HttpServletRequest request,HttpServletResponse response)throws ServletException, IOException
	{
		//Selected Process Page
		String idValue=request.getParameter("edit");
		if(idValue!=null && (!idValue.equals("")))
		{
			try
			{
				SearchProcessSheetAction spt=processActionFactory.getSearchProcessSheetAction();
				MouldingProcess process=spt.getMouldingProcess(idValue);
				request.setAttribute("process",process);
				RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/DisplayProcess.jsp");
				rd.forward(request, response);
			} catch (SQLException e)
			{
				getServletContext().log(e.getMessage());
				e.printStackTrace();
				throw new ServletException("Database not available");			
			}
			
		}
		else // No Process selected return to ProcessSearchPage
		{
			response.sendRedirect(request.getContextPath()+"/ProcessSheet_search");
		}
	}
	
	private void searchProcessSheets(HttpServletRequest request,HttpServletResponse response) throws ServletException, IOException
	{
		//check to if in search or edit mode TODO add delete mode
				String mode=request.getParameter("mode");
				//Passed to use as a search term
				String searchWord=request.getParameter("search");
				//Passed if an entry is to be edited
				String idValue=request.getParameter("edit");
				
				//Debug
				logger.debug(String.format("searchProcessSheets:mode:[%s] searchWord:[%s] ID:[%s]%n", mode,searchWord,idValue));
				
				
				
				//create an action
				SearchProcessSheetAction spt=processActionFactory.getSearchProcessSheetAction();
				String dispatchURL=null;
				try
				{
					//if the page is to do a search
					if(mode==null || mode.equals("search"))
					{
						List<MouldingProcess> list=null;
						//To check to search for all entries or entries where name=searchWord
						if(searchWord==null||searchWord.equals(""))// search for all entries
						{
							list=spt.search();
						}
						else //search for entry where name=searchWord 
						{
							
							list=spt.search("partId",searchWord);
						}
						request.setAttribute("processSheets", list); //Add the result list to the request object to be used by the JSP page
						//debug
						logger.debug(String.format("%d results returned %n",list.size()));
					
						dispatchURL="/WEB-INF/JSP/ProcessSheetSearchPage.jsp";
						
					}
//					else if(mode!=null && mode.equals("edit") && idValue!=null)// Edit mode
//					{
//						//open the JobTemplate JSPage in edit mode
//						JobTemplate job=sjt.getJobTemplate(idValue);
//						dispatchURL="/JSP/JobTemplate.jsp";
//						request.setAttribute("form", job);
//						request.setAttribute("mode","edit");
//						
//					}
//					else if(mode!=null && mode.equals("add"))
//					{
//						//open the JobTemplate JSPage in add mode
//						dispatchURL="/JSP/JobTemplate.jsp";
//					}
					else if(mode!=null)
					{
						if(mode.equals("add")||idValue==null) //idValue will equal null if the checked box isn't selected
						{
							logger.debug(String.format("searchProcessSheets:Opening ProcessPage.jsp"));
							//open the JobTemplate JSPage in add mode
							MouldingProcess process =new MouldingProcess();
							request.setAttribute("form", process);
							dispatchURL="/WEB-INF/JSP/ProcessPage.jsp";
						}
						else if(mode.equals("edit")&&idValue!=null)
						{
							//open the JobTemplate JSPage in edit mode
							logger.debug(String.format("searchProcessSheets:Opening ProcessPage.jsp in edit mode"));
							MouldingProcess process=spt.getMouldingProcess(idValue);
							dispatchURL="/WEB-INF/JSP/ProcessPage.jsp";
							request.setAttribute("form", process);
							request.setAttribute("mode","edit");
						}
					}
					//Get List of Material
					Map<Integer,Material> materials=materialActionFactory.getSearchMaterialAction().search();

					request.setAttribute("materials", materials);
					logger.debug(String.format("Materials:"+materials));
					
					RequestDispatcher rd=request.getRequestDispatcher(dispatchURL);
					rd.forward(request, response);
				}
				catch(SQLException se)
				{
					se.printStackTrace();
					throw new ServletException("Database not available:"+se.getMessage());
				}
		
	}

	/*
	 * Required by Spring
	 */
	@Autowired
	public void setProcessActionFactory(ProcessActionFactory processActionFactory)
	{
		this.processActionFactory = processActionFactory;
	}
	
	@Autowired
	public void setMaterialActionFactory(MaterialActionFactory materialActionFactory)
	{
		this.materialActionFactory = materialActionFactory;
	}
	
	@Override
	public void init() throws ServletException
	{
		WebApplicationContext context2=(WebApplicationContext)getServletContext().getAttribute("org.springframework.web.context.WebApplicationContext.ROOT");
		setProcessActionFactory((ProcessActionFactory)context2.getBean("processActionFactory"));
		setMaterialActionFactory((MaterialActionFactory)context2.getBean("materialActionFactory"));
		super.init();
	}
	
}

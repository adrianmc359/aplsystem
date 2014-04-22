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
import org.amc.servlet.action.SaveMaterialAction;
import org.amc.servlet.action.SearchMaterialAction;
import org.amc.servlet.model.Material;
import org.amc.servlet.model.MaterialForm;
import org.amc.servlet.validator.MaterialForm_Validator;
import org.apache.log4j.Logger;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.context.WebApplicationContext;

/**
 * Servlet implementation class APLMaterialServlet
 */
@WebServlet(
		description = "To Process Material Requests", 
		urlPatterns = { 
				"/Material_display",
				"/Material_save",
				"/Material_search",
				"/MaterialServlet"
				}
		,loadOnStartup=3)
public class APLMaterialServlet extends HttpServlet 
{
	private static final long serialVersionUID = 5984908504L;
	
	private static Logger logger=Logger.getLogger(APLMaterialServlet.class);
	
	private MaterialActionFactory materialActionFactory;
    /**
     * @see HttpServlet#HttpServlet()
     */
    public APLMaterialServlet() 
    {
        super();
        // TODO Auto-generated constructor stub
    }

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
	
	/**
	 * 
	 * @param request
	 * @param response
	 * @throws ServletException
	 * @throws IOException
	 */
	private void process(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		String referal=request.getRequestURI();
		logger.debug(String.format(referal));
		
		
		//Handle JobTemplate Page
		if(referal.endsWith("Material_save"))
		{
			saveMaterial(request, response);
		}
		else if(referal.endsWith("Material_display"))
		{
			displayMaterial(request, response);
		}
		else if(referal.endsWith("Material_search"))
		{
			searchMaterial(request, response);
		}
		else if(referal.endsWith("MaterialServlet"))
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
	
	private void saveMaterial(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		logger.debug(String.format("Context Path:"+request.getContextPath()));
		//check if page is in create or edit mode
		String mode=request.getParameter("mode");
		logger.debug(String.format("SaveMaterial:mode:[%s]%n", mode));//debug
		//create form
		
		//If an ID parameter is passed add it to the form
		MaterialForm jForm=(MaterialForm)request.getAttribute("material");
		logger.debug(String.format("\nSubmitted Process:"+jForm));

		
		//Validate Form
		MaterialForm_Validator validator=new MaterialForm_Validator();
		List<String> errors=validator.validate(jForm);
		
		//Check if user is a role to allow changes to the database
		if(!(request.isUserInRole("qc")||(request.isUserInRole("manager"))))
		{
				errors.add("User has no permissions to alter Material definitions!");
		}
		
		//If form validates with no errors
		if(errors.isEmpty())
		{
		
			//create model
			Material material;
			
			//String dispatcherURL="";
			
			SaveMaterialAction action=materialActionFactory.getSaveMaterialAction();
			try
			{
				material=MaterialForm.getMaterial(jForm);
				// New JobTemplate to Database
				if(mode==null||mode.equals("Enter"))
				{
					logger.debug(String.format("SaveMaterial:Entering entry into database"));
					action.save(material);
					response.sendRedirect(request.getContextPath()+"/Material_search"); // Goto the Search Window
					return; // Exit function 
				}
				else if(mode.equals("edit"))
				{
					logger.debug(String.format("SaveMaterial:Editing entry into database"));
					//Current JobTemplate is updated in the Database
					//processSheet.setId(Integer.parseInt(jForm.getId()));
					material.setId(Integer.parseInt(jForm.getId()));
					action.edit(material);
					response.sendRedirect(request.getContextPath()+"/Material_search"); // Goto the Search Window
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
			//if the form doesn't validate without errors then
			if(mode.equals("edit")) //Remember page is in edit mode
			{
				request.setAttribute("mode", mode);
			}
			request.setAttribute("errors", errors);
			request.setAttribute("form", jForm);			
			
			RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/Material.jsp");
			rd.forward(request, response);
		}
	}
	private void displayMaterial(HttpServletRequest request,HttpServletResponse response)throws ServletException, IOException
	{
		//Selected Process Page
		String idValue=request.getParameter("edit");
		if(idValue!=null && (!idValue.equals("")))
		{
			try
			{
				SearchMaterialAction spt=materialActionFactory.getSearchMaterialAction();
				Material process=spt.getMaterial(idValue);
				request.setAttribute("process",process);
				RequestDispatcher rd=request.getRequestDispatcher("/WEB-INF/JSP/Material.jsp");
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
			response.sendRedirect(request.getContextPath()+"/Material_search");
		}
	}
	
	private void searchMaterial(HttpServletRequest request,HttpServletResponse response) throws ServletException, IOException
	{
		//check to if in search or edit mode TODO add delete mode
				String mode=request.getParameter("mode");
				//Passed to use as a search term
				String searchWord=request.getParameter("search");
				//Passed if an entry is to be edited
				String idValue=request.getParameter("edit");
				
				//Debug
				logger.debug(String.format("searchMaterial:mode:[%s] searchWord:[%s] ID:[%s]%n", mode,searchWord,idValue));
				
				
				
				//create an action
				SearchMaterialAction spt=materialActionFactory.getSearchMaterialAction();
				String dispatchURL=null;
				try
				{
					//if the page is to do a search
					if(mode==null || mode.equals("search"))
					{
						Map<Integer,Material> list=null;
						//To check to search for all entries or entries where name=searchWord
						if(searchWord==null||searchWord.equals(""))// search for all entries
						{
							list=spt.search();
						}
						else //search for entry where name=searchWord 
						{
							
							list=spt.search("type",searchWord);
						}
						request.setAttribute("materials", list); //Add the result list to the request object to be used by the JSP page
						//debug
						logger.debug(String.format("%d results returned %n",list.size()));
					
						dispatchURL="/WEB-INF/JSP/MaterialSearchPage.jsp";
						
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
							logger.debug(String.format("searchMaterial:Opening Material.jsp"));
							//open the JobTemplate JSPage in add mode
							Material material =new Material();
							request.setAttribute("form", material);
							dispatchURL="/WEB-INF/JSP/Material.jsp";
						}
						else if(mode.equals("edit")&&idValue!=null)
						{
							//open the JobTemplate JSPage in edit mode
							logger.debug(String.format("searchMaterial:Opening Material.jsp in edit mode"));
							Material material=spt.getMaterial(idValue);
							dispatchURL="/WEB-INF/JSP/Material.jsp";
							request.setAttribute("form", material);
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
	@Autowired
	public void setMaterialActionFactory(MaterialActionFactory materialActionFactory)
	{
		this.materialActionFactory = materialActionFactory;
	}
	
	@Override
	public void init() throws ServletException
	{
		WebApplicationContext context2=(WebApplicationContext)getServletContext().getAttribute("org.springframework.web.context.WebApplicationContext.ROOT");
		setMaterialActionFactory((MaterialActionFactory)context2.getBean("materialActionFactory"));
		super.init();
	}
}

package myservlet;
import java.io.IOException;

import javax.servlet.RequestDispatcher;
import javax.servlet.ServletConfig;
import javax.servlet.ServletContext;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;

import org.amc.servlet.APLSystemServlet;
import org.amc.servlet.action.JobActionFactory;
import org.amc.servlet.action.SaveJobTemplateAction;
import org.amc.servlet.model.JobTemplate;
import org.junit.*;
import org.springframework.context.ApplicationContext;
import org.springframework.context.support.FileSystemXmlApplicationContext;
import org.springframework.web.context.WebApplicationContext;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class TestAPLServlet 
{
	private APLSystemServlet apl;
	private ApplicationContext context;
	private ServletContext sContext;
	private ServletConfig config;
	
	public TestAPLServlet()
	{
		//Spring ApplicationContext
		context= new FileSystemXmlApplicationContext("/src/main/webapp/WEB-INF/PartsContext.xml");
		//Mock Servlet Context
		ServletContext sContext= mock(ServletContext.class);
		when(sContext.getContextPath()).thenReturn("/myservlet");
		//Returns Spring Context
		when(sContext.getAttribute("org.springframework.web.context.WebApplicationContext.ROOT")).thenReturn(context);
		//Mock ServletConfig
		config = mock(ServletConfig.class);
		when(config.getServletContext()).thenReturn(sContext);
	}
	
	
	@Test
	public void testAPLSystemServlet()
	{
		
		HttpServletRequest request = mock(HttpServletRequest.class);       
        HttpServletResponse response = mock(HttpServletResponse.class);
        RequestDispatcher dispatcher = mock(RequestDispatcher.class);
        when(request.getRequestURI()).thenReturn("/myservlet/APLSystemServlet");
        when(request.getRequestDispatcher("/JSP/Main.jsp")).thenReturn(dispatcher);
        try
        {
        	apl =new APLSystemServlet();
        	apl.init(config);
        	apl.doGet(request, response);
        	apl.destroy();
        	verify(dispatcher).forward(request, response);
        	
        }
        catch(IOException e)
        {
        	
        }
        catch(ServletException se)
        {
        	
        }        
	}
	
	@Test
	public void testJobTemplate_display()
	{
		
		HttpServletRequest request = mock(HttpServletRequest.class);       
        HttpServletResponse response = mock(HttpServletResponse.class);
        RequestDispatcher dispatcher = mock(RequestDispatcher.class);
        when(request.getRequestURI()).thenReturn("/myservlet/JobTemplate_display");
        when(request.getRequestDispatcher("/JSP/JobTemplate.jsp")).thenReturn(dispatcher);
        try
        {
        	apl =new APLSystemServlet();
        	apl.init(config);
        	apl.doGet(request, response);
        	apl.destroy();
        	verify(dispatcher).forward(request, response);
        	
        }
        catch(IOException e)
        {
        	
        }
        catch(ServletException se)
        {
        	
        }        
	}
	
	@Test
	public void testLogout()
	{
		
		
		
		
		HttpServletRequest request = mock(HttpServletRequest.class);       
        HttpServletResponse response = mock(HttpServletResponse.class);
        HttpSession session= mock(HttpSession.class);
        
        when(request.getRequestURI()).thenReturn("/myservlet/logout");
        
       
        
        when(request.getSession()).thenReturn(session);
        
        
        
        try
        {
        	
        	
        	apl =new APLSystemServlet();
        	apl.setJobActionFactory((JobActionFactory)context.getBean("jobActionFactory"));
        	apl.init(config);
        	apl.doGet(request, response);
        	apl.destroy();
        	verify(request).logout();
        	
        }
        catch(IOException e)
        {
        	
        }
        catch(ServletException se)
        {
        	
        }        
	}
	
	
	//TODO Implement
	@Test
	public void testJobTemplate_save()
	{
		HttpServletRequest request = mock(HttpServletRequest.class);       
        HttpServletResponse response = mock(HttpServletResponse.class);
        RequestDispatcher dispatcher = mock(RequestDispatcher.class);
  
        when(request.getRequestURI()).thenReturn("/myservlet/JobTemplate_save");
        
        //Enter mode
        when(request.getParameter("mode")).thenReturn("Enter");
        
		
        when(request.getParameter("name")).thenReturn("Pot");
		when(request.getParameter("colour")).thenReturn("Green");;
		//checkedbox
		when(request.getParameter("external")).thenReturn("true");		
		when(request.getParameter("part_id")).thenReturn("Part:494949");
		when(request.getParameter("qss_no")).thenReturn("QSS TA22");
		when(request.getParameter("revision")).thenReturn("2");
		when(request.getParameter("version")).thenReturn("60g");
		when(request.getParameter("company")).thenReturn("Tosara");
		
		when(request.getRequestDispatcher("/JSP/JobTemplate.jsp")).thenReturn(dispatcher);
        
//        
//        try
//        {
//        	apl =new APLSystemServlet();
//        	apl.init(config);
//        	apl.doGet(request, response);
//        	apl.destroy();
//        	verify(request).setAttribute(eq("form"),any(JobTemplate.class));
//        	verify(request).setAttribute(eq("result"),anyString());
//        	
//        }
//        catch(IOException e)
//        {
//        	
//        }
//        catch(ServletException se)
//        {
//        	
//        }        
	}
	
}

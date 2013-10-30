package org.amc.servlet.dao;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

import org.amc.servlet.model.JobTemplate;

public class JobTemplateDAOImpl extends BasicDAO implements JobTemplateDAO 
{

	private static String tablename="jobtemplate";
	/* (non-Javadoc)
	 * @see org.amc.servlet.dao.JobTemplateDAO#addJobTemplate(org.amc.servlet.model.JobTemplate)
	 */
	@Override
	public void addJobTemplate(JobTemplate job) throws SQLException
	{
		//id,name,company,colour,external,part_id,qss_no, revision,version
		Connection connection=getConnection();
		PreparedStatement statement=connection.prepareStatement("INSERT INTO "+tablename+" VALUES(NULL,?,?,?,?,?,?,?,?)");
		statement.setString(1, job.getName());
		statement.setString(2, job.getCompany());
		statement.setString(3, job.getColour());
		statement.setBoolean(4, job.getExternal());
		statement.setString(5, job.getPart_id());
		statement.setString(6, job.getQss_no());
		statement.setString(7, job.getRevision());
		statement.setString(8, job.getVersion());
		
		statement.executeUpdate();
		closeDBObjects(null, statement, connection);
	}
	
	/* (non-Javadoc)
	 * @see org.amc.servlet.dao.JobTemplateDAO#updateJobTemplate(org.amc.servlet.model.JobTemplate)
	 */
	@Override
	public void updateJobTemplate(JobTemplate job) throws SQLException
	{
		//id,name,company,colour,external,part_id,qss_no, revision,version
		Connection connection=getConnection();
		PreparedStatement statement=connection.prepareStatement("UPDATE "+tablename+" set name=?,"
				+ "company=?,"
				+ "colour=?,"
				+ "external=?,"
				+ "part_id=?,"
				+ "qss_no=?,"
				+ "revision=?,"
				+ "version=? where id=?");
				statement.setString(1, job.getName());
				statement.setString(2, job.getCompany());
				statement.setString(3, job.getColour());
				statement.setBoolean(4, job.getExternal());
				statement.setString(5, job.getPart_id());
				statement.setString(6, job.getQss_no());
				statement.setString(7, job.getRevision());
				statement.setString(8, job.getVersion());
				statement.setString(9, String.valueOf(job.getId()));
				
	
				statement.executeUpdate();
				closeDBObjects(null, statement, connection);
		
	}
	
	/* (non-Javadoc)
	 * @see org.amc.servlet.dao.JobTemplateDAO#deleteJobTemplate(org.amc.servlet.model.JobTemplate)
	 */
	@Override
	public void deleteJobTemplate(JobTemplate job)
	{
		
	}
	
	/* (non-Javadoc)
	 * @see org.amc.servlet.dao.JobTemplateDAO#getJobTemplate(int)
	 */
	@Override
	public JobTemplate getJobTemplate(String jobTemplateId) throws SQLException
	{
		Connection connection=getConnection();
		PreparedStatement statement=connection.prepareStatement("select * from "+tablename+" where id=?;");
		statement.setString(1, jobTemplateId);
		ResultSet rs=statement.executeQuery();
		JobTemplate tempJob=null;
		if(rs.next())
		{
			tempJob=getJobTemplate(rs);
		}
		closeDBObjects(rs, statement, connection);
		return tempJob;
	}
	
	/* (non-Javadoc)
	 * @see org.amc.servlet.dao.JobTemplateDAO#findJobTemplates(java.lang.String, java.lang.String)
	 */
	@Override
	public List<JobTemplate> findJobTemplates(String col,String value) throws SQLException
	{
		Connection connection=getConnection();
		PreparedStatement statement=connection.prepareStatement("select * from "+tablename+" where "+col+"=?;");
		
		statement.setString(1, value);
		ResultSet rs=statement.executeQuery();
		List<JobTemplate> list=new ArrayList<JobTemplate>();
		
		while(rs.next())
		{
			JobTemplate tempJob=getJobTemplate(rs);
			list.add(tempJob);
		}
		closeDBObjects(rs, statement, connection);
		
		return list;
		
	}

	@Override
	public List<JobTemplate> findJobTemplates() throws SQLException 
	{
		Connection connection=getConnection();
		Statement statement=connection.createStatement();
		ResultSet rs=statement.executeQuery("select * from "+tablename+";");
		List<JobTemplate> list=new ArrayList<JobTemplate>();
		
		while(rs.next())
		{
			JobTemplate tempJob=getJobTemplate(rs);
			list.add(tempJob);
		}
		
		closeDBObjects(rs, statement, connection);
		return list;
	}
	
	
	//Don't call next or close the ResultSet
	private JobTemplate getJobTemplate(ResultSet rs) throws SQLException
	{
			JobTemplate tempJob=new JobTemplate(
					rs.getString("name"),
					rs.getString("part_id"),
					rs.getString("company"),
					rs.getString("version"),
					rs.getString("revision"),
					rs.getString("colour"),
					rs.getBoolean("external"),
					rs.getString("qss_no")
					);
			tempJob.setId(rs.getInt("ID"));
		return tempJob;	
		
	}
}
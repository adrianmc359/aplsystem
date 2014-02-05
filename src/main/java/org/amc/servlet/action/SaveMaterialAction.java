package org.amc.servlet.action;

import java.sql.SQLException;

import org.amc.servlet.dao.MaterialDAO;
import org.amc.servlet.dao.MouldingProcessDAO;
import org.amc.servlet.model.Material;
import org.amc.servlet.model.MouldingProcess;
import org.springframework.beans.factory.annotation.Autowired;

public class SaveMaterialAction 
{
	private MaterialDAO materialDAO;
	
	@Autowired
	public SaveMaterialAction(MaterialDAO materialDAO)
	{
		this.materialDAO=materialDAO;
	}
	/**
	 * Saves Job to the database as a new entry
	 * @param job
	 * @throws SQLException
	 */
	public void save(Material material) throws SQLException
	{
		this.materialDAO.addMaterial(material);
		
		
	}
	
	/**
	 * Updates database entry
	 * @param job
	 * @throws SQLException
	 */
	public void edit(Material material) throws SQLException
	{
		this.materialDAO.updateMaterial(material);
	}
}

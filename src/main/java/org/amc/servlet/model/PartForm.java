package org.amc.servlet.model;
/**
 * 
 * @author Adrian Mclaughlin
 * @version 1
 */
public class PartForm 
{
	String id;
	
	String colour;
	String company;
	boolean external;
	String name;
	String part_id;
	String qss_no;
	String revision;
	String version;
	
	public String getColour() {
		return colour;
	}
	public void setColour(String colour) {
		this.colour = colour;
	}
	public String getCompany() {
		return company;
	}
	public void setCompany(String company) {
		this.company = company;
	}
	public boolean getExternal() {
		return external;
	}
	public void setExternal(boolean external) {
		this.external = external;
	}
	public String getName() {
		return name;
	}
	public void setName(String name) {
		this.name = name;
	}
	public String getPart_id() {
		return part_id;
	}
	public void setPart_id(String part_id) {
		this.part_id = part_id;
	}
	public String getQss_no() {
		return qss_no;
	}
	public void setQss_no(String qss_no) {
		this.qss_no = qss_no;
	}
	public String getRevision() {
		return revision;
	}
	public void setRevision(String revision) {
		this.revision = revision;
	}
	public String getVersion() {
		return version;
	}
	public void setVersion(String version) {
		this.version = version;
	}
	public String getId() {
		return id;
	}
	public void setId(String id) {
		this.id = id;
	}
}


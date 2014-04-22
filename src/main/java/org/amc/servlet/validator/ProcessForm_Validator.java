package org.amc.servlet.validator;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import org.amc.servlet.model.MouldingProcess;
import org.amc.servlet.model.MouldingProcessForm;

public class ProcessForm_Validator 
{
	public List<String> validate(MouldingProcessForm form)
	{
		List<String> errors=new ArrayList<String>();
		
		String id=form.getId();
		try
		{
			Integer.parseInt(id);
		}
		catch(NumberFormatException nfe)
		{
			errors.add("ID is not an integer");
		}
		
		String productName=form.getPartId();
		if(productName==null||productName.equals(""))
		{
			errors.add("Product Name required");
		}
		
		String date=form.getDateOfIssue();
		try
		{
			java.sql.Date.valueOf(date);
		}
		catch(IllegalArgumentException iae)
		{
			errors.add("Date is not in the correct format:yyyy-mm-dd");
		}
		String machineSize=form.getMachineSize();
		try
		{
			Integer.parseInt(machineSize);
		}
		catch(NumberFormatException nfe)
		{
			errors.add("Machine Size is not an integer");
		}
		
		for(int i=9;i<81;i++)
		{
			String field=null;
			try
			{
				field=MouldingProcess.fields[i];
				Field refField=MouldingProcessForm.class.getDeclaredField(field);
				Float.parseFloat((String)refField.get(form));
				
			}
			catch(NoSuchFieldException nsfe)
			{
				nsfe.printStackTrace();
			}
			catch(NumberFormatException cce)
			{
				errors.add(field+" must be a number");
			}
			catch(IllegalAccessException iae)
			{
				iae.printStackTrace();
			}
			catch(NullPointerException npe)
			{
				errors.add(field+" must be a number(null)");
			}
		}
		for(int i=83;i<103;i++)
		{
			String field=null;
			try
			{
				field=MouldingProcess.fields[i];
				Field refField=MouldingProcessForm.class.getDeclaredField(field);
				Float.parseFloat((String)refField.get(form));
				
			}
			catch(NoSuchFieldException nsfe)
			{
				nsfe.printStackTrace();
			}
			catch(NumberFormatException cce)
			{
				errors.add(field+" must be a number");
			}
			catch(IllegalAccessException iae)
			{
				iae.printStackTrace();
			}
			catch(NullPointerException npe)
			{
				errors.add(field+" must be a number(null)");
			}
		}
//		String name=form.getName();
//		String colour=form.getColour();
//		String company=form.getCompany();
//		
//		
//		
//		if(name==null||name.trim().isEmpty())
//		{
//			errors.add("Product must have a name");
//		}
//		
//		if(colour==null||colour.trim().isEmpty())
//		{
//			errors.add("Product must have a colour");
//		}
//		
//		if(company==null||company.trim().isEmpty())
//		{
//			errors.add("Product must have a company associated with it");
//		}
//		
//		
//		
		
		return errors;
	}
}

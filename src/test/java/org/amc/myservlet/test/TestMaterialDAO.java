package org.amc.myservlet.test;

import static org.junit.Assert.*;

import java.util.Collection;
import java.util.Map;

import javax.persistence.EntityManager;
import javax.persistence.EntityManagerFactory;
import javax.persistence.Persistence;
import javax.persistence.Query;

import org.amc.dao.MaterialDAO;
import org.amc.model.Material;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
/**
 * 
 * @author Adrian Mclaughlin
 * @version 1
 */
public class TestMaterialDAO
{

	private EntityManager em;
	private EntityManagerFactory factory;
	private final String NAME="Moplen550";
	private final String COMPANY="TOSARA";
	private final String TYPE="ABS";
	
	private Material testMaterial;
	@Before
	public void setUp()
	{
		//Set up test Material
		testMaterial=new Material();
		
		testMaterial.setCompany(COMPANY);
		testMaterial.setName(NAME);
		testMaterial.setType(TYPE);
		
		factory=Persistence.createEntityManagerFactory("myDataSource");
		em=factory.createEntityManager();
		
		//Clear the table
		Query q=em.createNativeQuery("DELETE FROM processSheets");
		Query q1=em.createNativeQuery("DELETE FROM material");
		em.getTransaction().begin();
		q.executeUpdate();
		q1.executeUpdate();
		em.getTransaction().commit();
		
		
	}
	
	@After
	public void tearDown()
	{
		em.close();
		factory.close();
	}
	/**
	 * Test getMaterial method as well as addMaterial
	 */
	@Test
	public void testAddMaterial()
	{
		MaterialDAO d=new MaterialDAO(factory);
		d.addMaterial(testMaterial);
		
		Material actual=d.getMaterial(String.valueOf(testMaterial.getId()));
		assertEquals(testMaterial, actual);
	}

	@Test
	public void testUpdateMaterial()
	{	
		//Create Material DAO
		MaterialDAO d=new MaterialDAO(factory);
		//d.setEm(em);
		//Add Material Database
		d.addMaterial(testMaterial);
		
		//Check Material has been added and retrived
		Map<Integer,Material> list=d.findMaterials("name", NAME);
		Collection<Material> c=list.values();
		for(Material tm:c)
		{
			System.out.println(tm);
			if(tm.getName().equals(NAME))
			{
				tm.setType(("TEST"));
				d.updateMaterial(tm);
			}
		}
		assertNotSame(c.size(),0);
		
	}

	@Test
	public void testFindMaterialsStringString()
	{
		MaterialDAO d=new MaterialDAO(factory);
		//d.setEm(em);
		d.addMaterial(testMaterial);
		Map<Integer,Material> mp=d.findMaterials("name",NAME);
		assertEquals(mp.size(),1);
	}

	@Test
	public void testFindMaterials()
	{
		MaterialDAO d=new MaterialDAO(factory);
		//d.setEm(em);
		d.addMaterial(testMaterial);
		Map<Integer,Material> mp=d.findMaterials();
		assertEquals(mp.size(),1);
	}

}

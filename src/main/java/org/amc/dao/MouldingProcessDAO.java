package org.amc.dao;

import java.io.Serializable;
import java.util.List;

import javax.persistence.EntityManager;
import javax.persistence.EntityManagerFactory;
import javax.persistence.Query;

import org.amc.dao.MouldingProcessDAO;
import org.amc.model.MouldingProcess;

/**
 * 
 * @author Adrian Mclaughlin
 * @version 1
 */
public class MouldingProcessDAO extends DAO implements Serializable
{
	private static final long serialVersionUID = 7577290113094820714L;

	public MouldingProcessDAO(EntityManagerFactory emf)
	{
		super(emf);
	}

	public void addProcessSheet(MouldingProcess process)
	{
		EntityManager em=getEntityManager();
		em.getTransaction().begin();
		em.persist(process);
		em.getTransaction().commit();
		em.close();
	}

	public void updateProcessSheet(MouldingProcess process)
	{
		EntityManager em=getEntityManager();
		em.getTransaction().begin();
		em.merge(process);
		em.getTransaction().commit();
		em.close();
	}

	public void deleteProcessSheet(MouldingProcess process)
	{

	}

	public MouldingProcess getProcessSheet(String processId)
	{
		EntityManager em=getEntityManager();
		Query q=getEntityManager().createQuery("Select x from MouldingProcess x where x.id="+processId+"");
		MouldingProcess mp = (MouldingProcess)q.getSingleResult();
		em.close();
		return mp;

	}

	public List<MouldingProcess> findProcessSheets(String col, String value)
	{
		EntityManager em=getEntityManager();
		Query q=getEntityManager().createQuery("Select x from MouldingProcess x where x."+col+"='"+value+"'");
		List<MouldingProcess> resultList=(List<MouldingProcess>)q.getResultList();
		em.close();
		return resultList;
	}

	public List<MouldingProcess> findProcessSheets() 
	{
		EntityManager em=getEntityManager();
		Query q=getEntityManager().createQuery("Select x from MouldingProcess x");
		List<MouldingProcess> resultList=(List<MouldingProcess>)q.getResultList();
		em.close();
		return resultList;
	}
}

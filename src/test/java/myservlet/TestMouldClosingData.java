package myservlet;
import org.junit.*;
import org.amc.servlet.model.*;
import static org.junit.Assert.*;

public class TestMouldClosingData 
{
	@Test
	public void testGetMouldClosingTimeData()
	{
		int i=0;
		while(i<1000)
		{
			MouldingProcess p=new MouldingProcess();
			float openlimit=420f;
			float mouldClosingLimitPos=openlimit-(20*(float)Math.random());
			float mouldClosedLimitPos=mouldClosingLimitPos-(200*(float)Math.random());
			float clsSlowPos=mouldClosedLimitPos-(100*(float)Math.random());
			float clsSPPos=clsSlowPos-(clsSlowPos*(float)Math.random());
			p.setMouldClosingOpenLimitPos(mouldClosingLimitPos);
			p.setMouldClosedLimitPos(mouldClosedLimitPos);
			p.setClsSlowPos(clsSlowPos);
			p.setClsSPPos(clsSPPos);
			
			float mouldClosingOpenLimitSpeed=420*(float)Math.random();
			float mouldClosedLimitSpeed=420*(float)Math.random();
			float clsSPSpeed=100*(float)Math.random();
			
			p.setMouldClosingOpenLimitSpeed(mouldClosingOpenLimitSpeed);
			p.setMouldClosedLimitSpeed(mouldClosedLimitSpeed);
			p.setClsSPSpeed(clsSPSpeed);
			
			float[][] data=MouldingProcessUtil.getMouldClosingTimeData(p);
			float result=0f;
			System.out.printf("%f %f % f %f - %f % f %f%n",
					mouldClosingLimitPos,
					mouldClosedLimitPos,
					clsSlowPos,clsSPPos,
					mouldClosingOpenLimitSpeed,
					mouldClosedLimitSpeed,
					clsSPSpeed);
			System.out.printf("Expected:%f + %f + %f + %f = %f%n",mouldClosingLimitPos-mouldClosedLimitPos,mouldClosedLimitPos-clsSlowPos,clsSlowPos-clsSPPos,clsSPPos,(mouldClosingLimitPos-mouldClosedLimitPos)+(mouldClosedLimitPos-clsSlowPos)+(clsSlowPos-clsSPPos)+(clsSPPos));
			System.out.print("Actual:");
			for(float[] t:data)
			{
				result+=t[1];
				System.out.printf("%f +",t[1]);
			}
			System.out.printf("=%f%n", result);	
			System.out.printf("(%d)Expected:%f,Actual:%f%n",i,mouldClosingLimitPos,result);
			assertEquals(mouldClosingLimitPos,result,0.01);
			i++;
		}
	}
	
}

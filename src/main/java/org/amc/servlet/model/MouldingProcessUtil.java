package org.amc.servlet.model;

public class MouldingProcessUtil
{
	private final static float CLAMP_CLOSING_ACCELERATION=1250f;
	private final static float CLAMP_CLOSING_DEACCELERATION=-1000f;
	
	
	public static float getTotalInjectionTime(MouldingProcess process)
	{
		float totalInjectionTime=0f;
		float[] position=
			{
				process.getShotSize()+process.getPosTran(),
				process.getInjSpeedPosition_1(),
				process.getInjSpeedPosition_2(),
				process.getInjSpeedPosition_3(),
				process.getInjSpeedPosition_4(),
				process.getInjSpeedPosition_5(),
				process.getInjSpeedPosition_6()
			};
		
		float[] speed=
			{
				process.getInjectionSpeed_1(),
				process.getInjectionSpeed_2(),
				process.getInjectionSpeed_3(),
				process.getInjectionSpeed_4(),
				process.getInjectionSpeed_5(),
				process.getInjectionSpeed_6()
			};
		
		for(int pointer=0;pointer<speed.length;pointer++)
		{
			if(position[pointer+1]!=0)
			{
				totalInjectionTime+=(position[pointer]-position[pointer+1])/speed[pointer];
				
			}
			else
			{
				totalInjectionTime+=(position[pointer]-process.getPosTran())/speed[pointer];
				break;
			}
		}
		return totalInjectionTime;
	}
	
	/**
	 * s=ut+1/2at^2
	 * @param intialVelocity
	 * @param timeInSeconds
	 * @param acceleration
	 * @return Distance
	 */
	private static float getDistance(float intialVelocity,float timeInSeconds,float acceleration)
	{
		float distance=(float)((intialVelocity*timeInSeconds)+(0.5*acceleration*Math.pow(timeInSeconds, 2)));
		return distance;
	}
	/**
	 * a=(v-u)/t
	 * @param intialVelocity
	 * @param finalVelocity
	 * @param timeInSeconds
	 * @return acceleration
	 */
	private static float getAcceleration(float intialVelocity,float finalVelocity,float timeInSeconds)
	{
		return (finalVelocity-intialVelocity)/timeInSeconds;
	}
	
	/**
	 * t=(v-u)/a
	 * @param intialVelocity
	 * @param finalVelocity
	 * @param acceleration
	 * @return time
	 */
	private static float getTime(float intialVelocity,float finalVelocity,float acceleration)
	{
		return (finalVelocity-intialVelocity)/acceleration;
	}
	
	/**
	 * t=sqrt(2*s/a)
	 * @param distance
	 * @param acceleration
	 * @return time
	 */
	private static float getTime(float distance,float acceleration)
	{
		
		return (float)Math.sqrt((2*distance)/Math.abs(acceleration));
	}
	
	public static float getMouldClosingTime(MouldingProcess process)
	{
		float[][] data=getMouldClosingTimeData(process);
		
		float result=0f;
		for(int i=0;i<data.length;i++)
		{
			result+=data[i][0];
		}
		return result;
	}
	
	
	
	private static void calculateChanges(float[] posData, float[] speedData,float[][] data)
	{
		int i=0;// Pointer for posData and speedData array
		int t=0;// Pointer for the result array: Data[][]
		float time=0f;
		float distance=0f;
		float speed=0f;
		//Calculate the areas of velocity change
		while(i<posData.length)
		{
			//Check if there's an acceleration or deceleration
			if(MouldingProcessUtil.getAcceleration(speed, speedData[i],2)>0)
			{
				//Calculate transitions in the graph
				time=getTime(speed,speedData[i],CLAMP_CLOSING_ACCELERATION);
				distance=getDistance(speed, time, CLAMP_CLOSING_ACCELERATION);
				if(distance>posData[i])
				{
					time=getTime(posData[i],CLAMP_CLOSING_ACCELERATION);
					distance=posData[i];
					speed=CLAMP_CLOSING_ACCELERATION*time;
					speedData[i]=speed;
					//time=0;distance=0;
				}
				speed=speedData[i];
				data[t][0]=time;
				data[t][1]=distance;
				data[t][2]=speed;
			}
			else
			{
				time=getTime(speed,speedData[i],CLAMP_CLOSING_DEACCELERATION);
				distance=getDistance(speed, time, CLAMP_CLOSING_DEACCELERATION);
				speed=speedData[i];
				data[t][0]=time;
				data[t][1]=distance;
				data[t][2]=speed;
			}
			i++;
			t=t+2;//Store result in every odd index in the result array so we can calculate the even index results later
			
			
		}
	}

	
	/**
	 * todo Fix the problem when acceleration/deceleration distance is greater than the pre-set distance
	 * @param process
	 * @return an array of floats [[time_1,distance_1],...]
	 */
	
	public static float[][] getMouldClosingTimeData(MouldingProcess process)
	{
		float[][] data=new float[8][3];
		/*
		 * Calculate the time taken from mould protect position to closed position
		 * Initial velocity is zero
		 * Distance: Mould protect position to zero = Mould protect position
		 * t=sqrt(2*s/a)
		 */
		float closingTime=(float)Math.sqrt((2*process.getClsSPPos())/CLAMP_CLOSING_ACCELERATION);
		data[data.length-1][0]=closingTime;
		data[data.length-1][1]=process.getClsSPPos();
		data[data.length-1][2]=(CLAMP_CLOSING_ACCELERATION*closingTime);
		
		/*
		 * Calculate the time taken to reach mould protect position where mould velocity is zero
		 * Initial velocity is CLS SP Speed
		 * Final velocity is zero (At mould protect position)
		 * Distance: Time taken to decelerate to zero velocity.
		 * Time/Distance is inclusive of the time at mould cls sp pos
		 */
		data[data.length-2][0]=getTime(process.getClsSPSpeed(), 0,CLAMP_CLOSING_DEACCELERATION);
		data[data.length-2][1]=getDistance(process.getClsSPSpeed(), data[data.length-2][0], CLAMP_CLOSING_DEACCELERATION);
		data[data.length-2][2]=0;
	
		//Distance at each set mould clsoing speed
		float[] posData={
				process.getMouldClosingOpenLimitPos()-process.getMouldClosedLimitPos(),
				process.getMouldClosedLimitPos()-process.getClsSlowPos(),
				process.getClsSlowPos()-process.getClsSPPos()
		};
		
		//Defined mould closing speed
		float[] speedData={
				process.getMouldClosingOpenLimitSpeed(),
				process.getMouldClosedLimitSpeed(),
				process.getClsSPSpeed()
		};
		
		calculateChanges(posData, speedData, data);
		
		//Reset Counters and temporary variables
		int i=0,t=1;//T=1 so we can work on the even index results in the results array (data[][])
		float distance=0,speed=0,time=0;
		while(i<posData.length)
		{
			distance=posData[i];
			speed=speedData[i];
			float distanceDelta=0f;
			/*
			 * The initial mould acceleration distance is included in the distance ClosingOpenLimitPos->ClosedLimitPos
			 * Between the initial acceleration and final deceleration the time/distance is halved between each 
			 * set distance
			 */
			if(i==0 || data[t-2][1]==0)
			{
				distanceDelta=data[t-1][1]+data[t+1][1]/2;
			}
			else
			/*
			 * The final deceleration to zero velocity at mould protect position
			 */
			if(i==posData.length-1)
			{
				distanceDelta=data[t-1][1]/2+data[t+1][1];
			
			}
			else
			{
				distanceDelta=data[t-1][1]/2+data[t+1][1]/2;
			}
			
			//If the distance change is greater than the distance of pre set position then distance is set to zero
			if(distanceDelta>distance)
			{
				if(i>1)
				{
					posData[i-1]=posData[i-1]+(distance-distanceDelta);
					posData[i]=posData[i]-(distance-distanceDelta);
					t=t-2;
					i=i-1;
					continue;
				}
				else
				{
//					float newTime=getTime(distance/2, CLAMP_CLOSING_ACCELERATION);
//					float newSpeed=newTime*CLAMP_CLOSING_ACCELERATION;
//					data[t-1][0]=newTime;
//					data[t-1][1]=distance/2;
//					data[t-1][2]=newSpeed;
//					
//					data[t][0]=0;
//					data[t][1]=0;
//					data[t][2]=newSpeed;
//					posData[i]=distance/2;
//					speedData[i]=newSpeed;
//					calculateChanges(posData, speedData, data);
//					i++;
//					t=t+2;
//					continue;
//					posData[i]=0;
//					speedData[i]=0;
//					data[t-1][0]=0;
//					data[t-1][1]=0;
//					data[t-1][2]=0;
					data[t][0]=0;
					data[t][1]=0;
					data[t][2]=speedData[i];
//					calculateChanges(posData, speedData, data);
					i++;
					t+=2;
					continue;
					
					
				}
			}
			else
			{
				distance-=distanceDelta;
			}
			//Calculate time and save in result array
			time=distance/speed;
			data[t][0]=time;
			data[t][1]=distance;
			data[t][2]=speed;
			i++;
			t=t+2;
		}
		

		distance=0f;
		time=0f;
		for(float[] f:data)
		{
			distance+=f[1];
			time+=f[0];
			System.out.printf("Distance(%.3f) Time(%.3f) Speed(%.3f)%n",f[1],f[0],f[2]);
			
		}
		System.out.printf("Distance(%.3f) Time(%.3f)",distance,time);

		
		
		return data;
	}
	
	
	public static void main(String[] args)
	{
		//5.434387 + 63.155975 + 187.617035 + 154.884644 = 411.092041
		MouldingProcess process=new MouldingProcess();
		process.setMouldClosingOpenLimitPos(411f);
		process.setMouldClosedLimitPos(350.57f);
		process.setClsSlowPos(342.41f);
		process.setClsSPPos(187.5f);
		
		
		process.setMouldClosingOpenLimitSpeed(400f);
		process.setMouldClosedLimitSpeed(198.1f);
		process.setClsSPSpeed(61.67f);
		
		MouldingProcessUtil.getMouldClosingTimeData(process);
		
		
	}
}



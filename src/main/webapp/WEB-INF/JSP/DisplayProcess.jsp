<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c"%>
<%@taglib prefix="tags" tagdir="/WEB-INF/tags" %>
<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>process: ${process.partId}</title>
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/General.css">

<SCRIPT>
//An array of Chart IDs
var charts=[
		"injectionTimePressureChart",
		"injectionTimeSpeedChart",
		"mouldclosingtimechart",
		"cycleTimeLineChart"	
          ];

//Function to display the correct chart
function hideInfo(element)
{

	console.log(element);
	var selectedChart=element.options[element.selectedIndex];
	var id=selectedChart.value;
	
	for(var i=0;i<charts.length;i++)
	{
		if(charts[i]!=id)
		{
			var element=document.getElementById(charts[i]);
			element.style.visibility="hidden";
			console.log("Element:"+charts[i]+" is hidden");	
		}
	}
	var element=document.getElementById(id);
	element.style.visibility="visible";
	console.log("Element:"+selectedChart+" is visible");
}
</SCRIPT>

<style>
.title 
{
	z-index: 1;
}

.navbox
{
	z-index: 2;
}

.informationbox
{
	position:absolute;
	border-style: solid;
	border-radius: 25px;
	background-color: white;
	margin-top:4px;
	margin-left:20px;
	margin-right:auto;
	top:90px;
	width:50%;
	left:5%;
	padding: 20px;
	overflow:auto;
	font-size:xx-large;
}

.header
{
	font-weight: bold;
}

.chart
{
	position:absolute;
	top:400px;
	padding-top: 5px;
	padding-bottom: 5px;
	width:95%;
	height:50%;
	margin-right:10px;
	margin-left:10px;
	visibility:hidden;
	
}

select
{
	position:fixed;
	top:100px;
	right:5px;
	font-size:xx-large;
	z-index:4;
}

#injectionTimePressureChart
{
	visibility: visible;
}
</style>
</head>
<body>
<DIV class="title">
<H1> Process:${process.partId }</H1>
</DIV>
<%@ include file="NavigationDiv.jspf" %>
<DIV class="informationbox">
<TABLE>
<TR><TD class="header">Machine Size:</TD><TD>${process.machineSize}</TD></TR>
<TR><TD class="header">Machine No.</TD><TD>${process.machineNo}</TD></TR>
<TR><TD class="header">Material:</TD><TD>${process.material}</TD></TR>
<TR><TD class="header">Masterbatch:</TD><TD>${process.masterbatchNo}</TD></TR>
<TR><TD class="header">Date of Issue:</TD><TD>${process.dateOfIssue}</TD></TR>
<TR><TD class="header">Signed of by:</TD><TD>${process.signOffBy}</TD></TR>
</TABLE>
</DIV>

<select id="chartSelect" onclick="hideInfo(this)">
	<option value="injectionTimePressureChart" >Injection Time/Pressure</option>
	<option value="injectionTimeSpeedChart" >Injection Time/Speed</option>
	<option value="mouldclosingtimechart" >Mould Closing</option>
	<option value="cycleTimeLineChart" >Cycle Time</option>
</select>

<tags:Chart>
	<jsp:attribute name="injectiontimespeed">
		<tags:InjectionTimeSpeed process="${process}"></tags:InjectionTimeSpeed>
	</jsp:attribute>
	<jsp:attribute name="injectiontimepressure">
		<tags:InjectionTimePressure process="${process}"></tags:InjectionTimePressure>
	</jsp:attribute>
	<jsp:attribute name="cycletimeline">
		<tags:CycleTimeLine process="${process}"></tags:CycleTimeLine>
	</jsp:attribute>
	<jsp:attribute name="mouldclosingtime">
		<tags:MouldClosing process="${process}"></tags:MouldClosing>
	</jsp:attribute>
</tags:Chart>

</body>
</html>
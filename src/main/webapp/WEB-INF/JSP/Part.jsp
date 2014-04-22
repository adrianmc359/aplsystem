<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@taglib  uri="http://java.sun.com/jsp/jstl/core" prefix="c"%>
<!DOCTYPE html>

<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>ACME Plastics:Part Description Page</title>
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/General.css">
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/SearchPage.css">
<SCRIPT>
function home()
{
	location="Part_search";
}
</SCRIPT>
<STYLE>

TABLE
{
	position:fixed;
	top:110px;
	background-color:white;
	width:95%;	
	margin-left:2%;
	margin-right:auto;

}

TD
{
	text-align:left;
	font-size:xx-large;
	line-height: 77px;
}

TH
{
	text-align:left;
	font-size:xx-large;
}

input[type="text"]
{
	width:70%;
	font-size:xx-large;
}

</STYLE>
<script src="${pageContext.request.contextPath}/js/InputFocus.js"></script>
</head>
<body>
<DIV class="title">
<H1> Part Description</H1>
</DIV>

<%@ include file="NavigationDiv.jspf" %>
<%-- Display errors if there any --%>
<c:if test="${errors ne null }">
<SCRIPT>alert("${errors}");</SCRIPT>
</c:if>
<%-- The Form to get the values for the new or edited JobTemplate Object--%>

<FORM method='post' action='./Part_save'>
<input type="hidden" name='id' <c:if test='${form ne null}'>value='${form.id}'</c:if>/>
<TABLE>
<%-- To be used in edit mode to store the id of the object being edited --%>
<TR><TD class="description">Name</TD><TD>
	<input type='text' name='name' <c:if test='${form ne null}'>value="<c:out value='${form.name}' />"</c:if> autofocus="autofocus"/>
</TD></TR>
<TR><TD class="description">Company</TD><TD>
	<input type='text' name='company' <c:if test='${form ne null}'>value="<c:out value='${form.company}' />"</c:if> />
</TD></TR>
<TR><TD class="description">Product No.</TD><TD>
	<input type='text' name='part_id' <c:if test='${form ne null}'>value="<c:out value='${form.part_id}' />"</c:if> />
</TD></TR>
<TR><TD class="description">Version</TD><TD>
	<input type='text' name='version' <c:if test='${form ne null}'>value="<c:out value='${form.version}' />"</c:if> />
</TD></TR>
<TR><TD class="description">Revision</TD><TD>
	<input type='text' name='revision' <c:if test='${form ne null}'>value="<c:out value='${form.revision}' />"</c:if> />
</TD></TR>
<TR><TD class="description">Colour</TD><TD>
	<input type='text' name='colour' <c:if test='${form ne null}'>value="<c:out value='${form.colour}' />"</c:if> />
</TD></TR>
<TR><TD class="description">External</TD><TD>
	<input type='checkbox' name='external' <c:if test='${form ne null and form.external eq true}'>checked='checked'</c:if> />
</TD></TR>
<TR><TD class="description">QSS No.</TD><TD>
	<input type='text' name='qss_no' <c:if test='${form ne null}'>value="<c:out value='${form.qss_no}' />"</c:if> />
</TD></TR>


</TABLE>

<span class="buttons">
<button type="button" value="home" onclick="home()">Home</button>
<%-- To tell the servlet which mode the page is submitting in --%>
<c:if test="${mode eq null }">
	<input type='submit'  name="mode" value='Enter'/>
</c:if>
<c:if test="${mode eq 'edit' }">
	<input type='submit'  name="mode" value='Edit'/>
</c:if>
</span>
</FORM>

<DIV class="result">
<%-- To let the user know the part has been saved --%>
<c:if test="${result ne null }">
Part:${result}<br/>
</c:if>
</DIV>

</body>
</html>
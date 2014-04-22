<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c" %>
<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/General.css">
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/SearchPage.css">
<title>ACME Plastics:Parts Search Page</title>
<STYLE>


</STYLE>
<script src="js/SearchPage.js"></script>
</head>
<body>
<DIV class="title">
<H1> Part Inventory</H1>
</DIV>
<%@ include file="NavigationDiv.jspf" %>

<FORM action="${pageContext.request.contextPath}/Part_search" method="post">
<DIV class="results">
<TABLE>
<TR><TH>Id</TH><TH>Company</TH><TH>Name</TH><TH>Version</TH><TH>Colour</TH><TH>QSS no.</TH><TH></TH></TR>
<c:forEach items="${parts}" var="part">
<TR  onclick="selected(this)"><TD><c:out value="${part.id}"/></TD><TD><c:out value="${part.company}"/></TD><TD><c:out value="${part.name}"/></TD><TD><c:out value="${part.version}"/></TD><TD><c:out value="${part.colour}"/></TD><TD><c:out value="${part.qss_no}"/></TD><TD  class="checkbox"><input type="checkbox" name="edit" value="${part.id}"/></TD></TR>
</c:forEach>
<TR><TD></TD><TD></TD><TD></TD><TD></TD><TD></TD><TD></TD><TD></TD></TR>
</TABLE>
</DIV>
<SPAN class="search">
<input type="text" name="search"/><input type="submit" name="mode" value="search"/>
</SPAN>
<SPAN class="buttons"><input type="submit" name="mode" value="add"/><input type="submit" name="mode" value="edit" onmouseout="enable(this)" onmouseover="isChecked(this)" /></SPAN>
</FORM>
</body>
</html>
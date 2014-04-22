<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c" %>
<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>ACME Plastics :Material Search Page</title>
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/General.css">
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/SearchPage.css">
<STYLE>

</STYLE>
<script src="js/SearchPage.js"></script>
</head>
<body>
<DIV class="title">
<H1>Material</H1>
</DIV>
<%@ include file="NavigationDiv.jspf" %>


<FORM action="${pageContext.request.contextPath}/Material_search" method="post">
<DIV class="results">
<TABLE>
<TR><TH>Company</TH><TH>Name</TH><TH>Type</TH><TH></TH></TR>
<c:forEach items="${materials}" var="material">
<TR  onclick="selected(this)"><TD><c:out value="${material.value.company}"/></TD><TD><c:out value="${material.value.name}"/></TD><TD><c:out value="${material.value.type}"/></TD><TD  class="checkbox"><input type="checkbox" name="edit" value="${material.value.id}"/></TD></TR>
</c:forEach>
<TR><TD></TD><TD></TD><TD></TD></TR>
</TABLE>
</DIV>
<SPAN class="search">
<input type="text" name="search"/><input type="submit" name="mode" value="search"/>
</SPAN>
<SPAN class="buttons"><input type="submit" name="mode" value="add"/><input type="submit" name="mode" value="edit" onmouseout="enable(this)" onmouseover="isChecked(this)" /></SPAN>
</FORM>

</body>
</html>
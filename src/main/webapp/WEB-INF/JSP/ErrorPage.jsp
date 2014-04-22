<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page isErrorPage="true" %>
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c" %>
<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>Error has occurred</title>
<link rel="stylesheet" type="text/css" href="${pageContext.request.contextPath}/css/General.css">
</head>
<body>
<%@ include file="NavigationDiv.jspf" %>
<DIV class="error_message">
${pageContext.exception}
<br/>
${pageContext.errorData.statusCode}
<br/>
${pageContext.errorData.requestURI}
<br/>
<br/>

<c:forEach var="trace" 
         items="${pageContext.exception.stackTrace}">
<p>${trace}</p>
</c:forEach>

</DIV>
</body>
</html>
<!--  
	@author Adrian Mclaughlin
 	@version 1
-->
<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c"%>

<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
</head>
<body>

<jsp:useBean id="processSheet" class="org.amc.servlet.model.MouldingProcessForm" scope="request"/>

<jsp:setProperty name="processSheet" property="*" />
<jsp:forward page="/ProcessSheet_save"/>
</body>
</html>
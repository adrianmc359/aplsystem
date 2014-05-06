<%@ tag language="java" pageEncoding="UTF-8"%>
<%@taglib  uri="http://java.sun.com/jsp/jstl/core" prefix="c"%>
<%@ attribute name="value" rtexprvalue="true" required="true" %>
<%@ attribute name="href" rtexprvalue="true" required="true" %>
<%@ attribute name="position" rtexprvalue="true" required="false" %>

<c:if test="${empty position}">
	<c:set var="position" value="100px"/>
</c:if>
<a href="${href}">
<div class="navbox" style="right:${position};">${value}</div>
</a>

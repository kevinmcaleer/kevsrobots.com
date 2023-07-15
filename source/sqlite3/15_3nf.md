---
title: Third Normal Form (3NF)
description: 3NF helps eliminate data redundancy and improves data integrity in relational databases
layout: lesson
---

> The Third Normal Form (3NF) is an important concept in database normalization. It helps eliminate data redundancy and improves data integrity in relational databases.
>
> 3NF is a level of normalization that builds upon the ideas of the First Normal Form (1NF) and the Second Normal Form (2NF). It focuses on ensuring that data is organized without any repeating groups or dependencies.
> 
> To achieve 3NF, the following conditions must be met:
>
> 1. The table must be in 2NF.
> 2. There should be no transitive dependencies between non-key attributes.
> 
> By eliminating transitive dependencies, 3NF helps prevent data update, insertion, and deletion anomalies. It ensures that changes to one attribute do not cause inconsistencies or redundancies in other parts of the table.
>
> Let's consider an example to understand how to normalize data to 3NF. Suppose we have a table called "Students" with the following columns: Student_ID, Student_Name, Course, and Course_Instructor.
>
> | Student_ID | Student_Name | Course         | Course_Instructor |
> |------------|--------------|----------------|------------------|
> | 1          | Alice        | Math           | Mr. Johnson      |
> | 2          | Bob          | Science        | Mrs. Smith       |
> | 3          | Alice        | English        | Mr. Johnson      |
> | 4          | Charlie      | Math           | Mrs. Brown       |
> {:class="table"}
>
> In this table, we have repeating groups of Course and Course_Instructor for each Student_Name. To normalize this data to 3NF, we can create two separate tables: "Students" and "Courses."
>
> **Table: Students**
>
> | Student_ID | Student_Name |
> |------------|--------------|
> | 1          | Alice        |
> | 2          | Bob          |
> | 3          | Alice        |
> | 4          | Charlie      |
> {:class="table"}
>
> **Table: Courses**
>
> | Student_ID | Course         | Course_Instructor |
> |------------|----------------|------------------|
> | 1          | Math           | Mr. Johnson      |
> | 2          | Science        | Mrs. Smith       |
> | 3          | English        | Mr. Johnson      |
> | 4          | Math           | Mrs. Brown       |
> {:class="table"}
>
> By splitting the original table into two tables, we remove the repeating groups and achieve 3NF. The "Students" table contains unique student information, while the "Courses" table contains course details associated with each student.
>
> 3NF helps ensure that data is organized efficiently, reduces redundancy, and minimizes the risk of data inconsistencies. By following 3NF guidelines, database designers can create well-structured and reliable relational databases.
{:class="bg-blue"}

---

---
title: Programming Language Generations
description: >-
    A gantt chart showing the generations of programming languages
layout: content
---

# {{page.title}}
## {{page.description}}

---

{% mermaid %}
gantt
    title Generations of Programming Languages
    dateFormat YYYY
    section 1st Generation
    Machine Language :1940, 1950
    section 2nd Generation
    Assembly Language :1951, 1960
    section 3rd Generation
    High-Level Languages (Fortran, COBOL) :1961, 1970
    C : 1972, 1980
    section 4th Generation
    SQL, Other 4GLs :1971, 1980
    C++ : 1983, 1990
    Python : 1991, 2000
    Java : 1995, 2005
    JavaScript : 1995, 2005
    section 5th Generation
    Logic Programming, AI Languages :1981, 1990
    Arduino : 2005, 2015
    Rust : 2010, 2020
    MicroPython : 2014, 2024

{% endmermaid %}

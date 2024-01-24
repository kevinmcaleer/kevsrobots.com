---
title: Customizing Your Jekyll Site
description: Learn how to personalize your Jekyll site with themes, custom CSS/HTML, and JavaScript.
layout: lesson
type: page
---

## Introduction

Welcome to Lesson 4. In this lesson, we'll explore the various ways you can customize your Jekyll site to make it truly your own. We'll look at themes, layouts, and how to add custom CSS and JavaScript for more personalized functionality.

---

## Part 1: Themes and Layouts

Jekyll themes provide a quick way to style your site. But, they are also fully customizable.

---

### Choosing and Applying a Theme

- You can find themes on sites like [jekyllthemes.org](https://jekyllthemes.org/).
- To apply a theme, modify the `theme` setting in your `_config.yml` file.

> ## _config.yml
>
> The `_config.yml` file is the main configuration file for your Jekyll site. It contains settings like the site's title, description, and theme. It's written in YAML, a human-readable data serialization language.
>
> Here is an example `_config.yml` file:
>
> ```yaml
> title: My Jekyll Site
> description: A site about Jekyll
> theme: jekyll-theme-minimal
>```

---

### Customizing Layouts

- Layouts define the structure of your pages.
- Found in the `_layouts` directory, you can edit them to change how pages and posts are structured.

---

## Part 2: Customizing CSS and HTML

Jekyll allows you to alter the CSS and HTML for finer control over your site’s appearance.

---

### Modifying CSS

- CSS files are usually found in the `assets/css` directory.
- You can add your own styles or override existing ones to change the look and feel of your site.

---

### Changing HTML Templates

- HTML templates are located in the `_layouts` and `_includes` directories.
- By modifying these files, you can change the structure of your site's pages.

---

## Part 3: Adding JavaScript and Interactive Elements

Enhance your site's interactivity and functionality with JavaScript.

---

### Integrating JavaScript

- Add custom JavaScript files in the `assets/js` directory.
- Include them in your site's HTML templates for global use.

---

### Adding Interactive Elements

- Examples include contact forms, image sliders, or dynamic content loading.
- Ensure you test these elements for performance and compatibility.

---

## Conclusion

You now have the tools to customize your Jekyll site, making it not only functional but also unique in design and interactivity.

---

## Additional Resources

- [Jekyll Themes](https://jekyllthemes.org/)
- [W3Schools for HTML/CSS](https://www.w3schools.com/)

---

## Assignment

Customize the theme of your Jekyll site. Try modifying the CSS to change the color scheme and add a simple JavaScript function, like a dynamic date display.

---

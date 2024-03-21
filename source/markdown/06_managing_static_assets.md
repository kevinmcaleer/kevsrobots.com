---
title: Managing Static Assets
description: A guide to effectively organizing and using images, CSS, and JavaScript in your Jekyll site to create a visually appealing and interactive experience.
layout: lesson
type: page
cover: /learn/jekyll/assets/managing_assets.jpg
---

![Managing Static Assets cover image]({{ page.cover }}){:class="cover"}

## Introduction

Static assets are crucial components of any website, contributing to its aesthetics, functionality, and overall user experience. In Jekyll, managing these assets efficiently is key to building a professional and polished site.

## Organizing Your Assets

Jekyll supports a flexible directory structure, allowing you to organize your assets in a way that makes sense for your project. A common approach is to create dedicated directories for each type of asset within the root of your Jekyll project:

- `images/` for all your image files.
- `css/` for your stylesheets.
- `js/` for your JavaScript files.

## Referencing Static Assets

To reference static assets in your Jekyll site, use the `site.baseurl` variable to ensure your paths are correct, especially when your site is hosted in a subpath of a domain.

### Images

```markdown
![An example image]({{ site.baseurl }}/images/example.jpg)
```

### CSS

Include a link to your CSS file in the `<head>` section of your layout template:

```html
<link rel="stylesheet" href="{{ site.baseurl }}/css/styles.css">
```

### JavaScript

Include your JavaScript files before the closing `</body>` tag in your layout template for better page load performance:

```html
<script src="{{ site.baseurl }}/js/scripts.js"></script>
```

## Using SASS/SCSS with Jekyll

Jekyll has built-in support for Sass, a CSS preprocessor that allows you to use variables, nested rules, mixins, and more in your stylesheets.

1. Store your `.scss` or `.sass` files in a `_sass` directory.
2. Create a main stylesheet in your `css` directory that uses the `@import` directive to include your Sass files. Ensure this file has a front matter section, even if it's empty, to tell Jekyll to process it.

```scss
---
---

@import "main";
```

## Practice Exercise

1. Organize your Jekyll project by creating `images`, `css`, and `js` directories.
2. Add a sample image, a CSS file with basic styles, and a simple JavaScript file that adds interactive functionality to your site.
3. Update your default layout to include these assets and view the changes locally.

## Additional Resources

- [Jekyll Assets Documentation](https://jekyllrb.com/docs/assets/)
- [Sass Basics](https://sass-lang.com/guide)

---

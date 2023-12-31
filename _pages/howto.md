---
layout: single
title: "How to Modify the Website"
permalink: /how2
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation

taxonomy: markup

my_variable: scripts.html
---

> Only one thing is impossible for God: To find any sense in any copyright law on the planet.
> <cite><a href="http://www.brainyquote.com/quotes/quotes/m/marktwain163473.html">Mark Twain</a></cite>

## Info

This website is created using [Minimal Mistakes Jekyll Theme](https://mmistakes.github.io/minimal-mistakes/). The guide on how to use this theme is well explained in the [Quick-Start Guide](https://mmistakes.github.io/minimal-mistakes/). The purpose of this page is to have a quick reference on some features which are mainly used in this website.

## How to modify an existing page

The sources of this website are written in markdown syntax. There are **3 steps** in modifying an existing page:

1. Find the respective file under `_pages/COURSENAME`.
1. Do necessary modifications either [locally](#local-edit) or [via github.com](#via-githubcom)
1. Save and push to GitHub. The changes will be automatically applied. You can follow the status at: [GitHub Actions](https://github.com/frdedynamics/hvl_robotics_website/actions)

## How to add a new page

There are **5 steps** in adding a page:

1. Find the respective folder under `_pages`.
1. Create the name accordingly such as `COURSENAME-LECTURENAME.md`
1. Copy-paste the course content header:

```markdown
---
layout: single
title: "ELE306 Lesson 1"
permalink: /courses/ele306/l1
toc: true # The right side bar "on this page"
breadcrumbs: true  # the directory-of-documents type of header navigation
sidebar:
  nav: "ele306"  # the left navigation bar. Choose which category you want.
taxonomy: markup
---
```

1. Change the **title**, **permalink** and **nav** elements.

- **Title:** is just the name of the page. It can be chosen freely.
- **Permalink:** is the link to this page. By using this link in anywhere in any page, you can create a reference to this page. For consistency between notes, please keep the rule `/courses/courseXXX/lX` for lecture notes, `/courses/courseXXX/aX` for assignments and `/courses/courseXXX/labX` for lab exercises.
- **Navigation:** is the category of this page belong to. This tag is important for the left navigation bar and `_data/navigation.yml`

1. Add the title and the permalink under the respective parent folder in `_data/navigation.yml`.

  ```markdown
  - title: "ELE306 Lesson-X"
    url: /courses/ele306/lX
  ```

### Where to edit files

You can edit files either locally or via github.com.

### Local Edit

1. You need `jekyll` installation. The steps are in the [link](https://jekyllrb.com/docs/installation/windows/).
1. You need to clone the [source codes of this website](https://github.com/frdedynamics/hvl_robotics_website).

- if it is the first time, you need to run `bundle install` via CMD terminal under the downloaded folder. (Your local directory for the website `/hvl_robotics_website`)

1. Do the desired changes in the respective `_pages/XXX.md` files.
1. If you want to see the changes before pushing to the cloud, you need to run `bundle exec jekyll serve`. The changes will immediately apply at **<http://127.0.0.1:4000/>** or **<http://localhost:4000/>**.

### via github.com

You don't need any installation for this method.

1. Browse to the page that you want to edit on [GitHub](https://github.com/frdedynamics/hvl_robotics_website).
1. Click *edit this file* button on the right-top corner.
1. Do the desired changes.
1. Save and push changes.

## Important files

{: .notice--info}
This section is for advanced users.

- All the source files are under `_pages/`. 99% of the time, you don't need to browse anywhere else than this.
- If you want to modify things on the **Home** page: `_posts/2023-06-24-home.md`
- The source code of this webpage: `_posts/2023-06-24-how-to-modify.md`
- The **_config.yml** file and **Gemfile** are responsible in all kind of dependencies and settings. The details can be found [here](https://jekyllrb.com/docs/structure/).

## Useful links

- Markdown cheat-sheet.
- Minimal mistakes theme source code: <https://github.com/mmistakes/minimal-mistakes>
- Tags and formatting: <https://mmistakes.github.io/minimal-mistakes/markup/markup-html-tags-and-formatting/>

## TODO

- ROS submodule.
- Most used markdown features in this website.

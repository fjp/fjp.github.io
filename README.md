# Blog Repository

This repository holds the data for my blog. It is based on the theme [minimal mistakes](https://mmistakes.github.io/minimal-mistakes/) which is defined in the 
It is automatically transformed by [Jekyll](http://github.com/mojombo/jekyll) into a static site whenever I push this repository to GitHub.
You can see the result at [fjp.github.io](fjp.github.io) or [fjp.at](https://fjp.at).

[![Tip Me via PayPal](https://img.shields.io/badge/PayPal-tip%20me-green.svg?logo=paypal)](https://www.paypal.me/fpucher)

# Setup on MacOS

The basic setup to get a similar site up and running is explained with the following steps.
For more details checkout https://jekyllrb.com/docs/github-pages/

Install ruby  
`brew install ruby` or on ubuntu `sudo apt-get install ruby`

Install Jekyll and Bundler gems through RubyGems  
`gem install jekyll bundler`

Create a new Jekyll site at ./myblog  
`jekyll new myblog`

Change into your new directory  
`cd myblog`

Build the site on the preview server  
`bundle exec jekyll serve`

- Now browse to http://localhost:4000
- Add it to a new github repository named username.github.io

```
git init
git add .
git add .gitignore
git commit -m "adds jekyll blog"
git remote add origin https://github.com/username/username.github.io.git
git push -u origin master
```
# Create New Posts


New posts are simply created in the _posts folder but the naming convention is important.
For more details go to https://jekyllrb.com/docs/structure/

Add the changes to github by adding, committing and pushing the new post file.


[![Support via PayPal](https://cdn.jsdelivr.net/gh/twolfson/paypal-github-button@1.0.0/dist/button.svg)](https://www.paypal.me/fpucher)

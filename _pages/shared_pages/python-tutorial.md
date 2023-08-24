<!-- # Python Tutorial -->

Disclaimer: Python is an extensive programming (or scripting) language. It is neither easy not effective to teach Python in a single webpage. The main purpose of this page is to create a shortcut to the commands that we use a lot during this course. We also provide links to nice Python tutorials if you are interested to improve your skills in Python programming.

{: .notice--info}
Please look at and admire this website: [https://docs.python.org](https://docs.python.org/3/tutorial/index.html)

# Running Python scripts

Let's start a simple Python script:

```python
print('Hello world')
```
## Through Terminal
You can write it directly in your terminal:

1. Open a terminal: **Ctrl + Alt + T**
1. Type: `python3`
1. Paste the code above.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/python-linux/terminal-python.png)

Or you can directly run Python scripts via terminal.

1. Open a terminal: **Ctrl + Alt + T**
2. Create a Python file: `gedit my_first_python.py`
3. Save and close.
4. Run: `python3 my_first_python.py`

## Through VSCode
You need an IDE for more complicated code. We use VSCode.

1. Open Visual Studio Code
2. File > New File > Python file
3. Paste the code above
4. Run (play button rigth above).

# Functions

Simple function:
```python
def main():
    print('Hello world')

if __name__ == '__main__':
    main()
```

Call a function inside another function:

```python
def main():
    my_print_function()

def my_print_function():
    print('Hello world')

if __name__ == '__main__':
    main()
```

Rarely you define a function inside a function (nested function):

```python
def outerFunction(text): 
    text = text 
    
    def innerFunction(): 
        print(text) 
    
    innerFunction() 
    
if __name__ == '__main__': 
    outerFunction('Hello world') 
```

# Imports
Imports are libraries.


# TODO:

- Simple print
- import and from
- functions
- import from other file
- os, **join** path: https://www.geeksforgeeks.org/python-os-path-join-method/
    - This is particularly important for **servo.py** and **XX.launch.py** files.
    - **glob**: https://docs.python.org/3/library/glob.html
    - https://www.geeksforgeeks.org/how-to-use-glob-function-to-find-files-recursively-in-python/
- classes
    - self
    - init constructor
    - methods
        - Just mention static and class methods
- parenting
    - super()
- Simple 3 exercises.

## Useful materials:
Python books
Websites
Youtube videos

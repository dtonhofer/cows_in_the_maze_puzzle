# "Cows in the Maze" puzzle solver

## What is this

Solve the "Cows in the Maze" puzzle from the section "Mathematical Recreations" of _Scientific American_, December 1996. 

The puzzle originally appeared in Robert Abbott's book _Supermazes_ (Prima Publishing, Rocklin, California, 1996) ([review](http://www.logicmazes.com/super.html)). There is a [Wikipedia reference](https://en.wikipedia.org/wiki/Robert_Abbott_(game_designer)#Where_Are_the_Cows?) for this puzzle. The author of this "Mathemtical Recreations" installment, Ian Stewart, has written a book with the [eponymous title "Cows in the Maze"](https://global.oup.com/academic/product/cows-in-the-maze-9780199562077?cc=lu&lang=en&).

This is an early coding effort, written in December 1996. In C++. Today I would do it in Prolog. I had modernize it a bit so that it compiles with GNU C++ Compiler. 

Here is the relevant page from _Scientific American_.

The task is:

   * You have two pencils.
   * One pencil starts in box 1, one in box 7.
   * Following the (nondeterministic) instructions so that at least one of the pencils ends up in the box labeled GOAL.

![Extract from Scientific American](/img/Scientific_American_Mathematical_Recreations_1996-12.png?raw=true "Mathematical Recreations")

## The solution

```
(.., 1, 7, )
(m., 2, 7, )
(m.,15, 7, )
(m.,40, 7, )
(m.,60, 7, )
(.m,60, 5, )
(m.,25, 5,*)
(m., 7, 5,*)
(m.,26, 5,*)
(m.,61, 5,*)
(.m,61, 2,*)
(.m,61,15,*)
(.m,61,40,*)
(.m,61,65,*)
(.m,61,75, )
(mm, 1, 1, )
(m., 9, 1, )
(m.,35, 1, )
(.m,35, 9, )
(.m,35,35, )
(m.,40,35, )
(m.,60,35, )
(m.,25,35,*)
(m., 7,35,*)
(m.,26,35,*)
(m.,61,35,*)
(m., 1,35,*)
(m., 9,35,*)
Goal state encountered at (m., 2,35,*)
```

# CREATE_ELLIPSE

## create_ellipse.cpp
In this [file](/create_ellipse/create_ellipse.cpp) I played around with creating the templates and finding the eye center (make_template(), makeEllipse()) and the eyelids (points(), eyelid()).

The points() function returns all the points on an arc defining an eyelid. These points can later be used to fit a polynomial to define the eyelid.

## ellipse.ipynb
[File](/create_ellipse/ellipse.ipynb) for trying to undesrtand the ellipses & deformation.

## polyfit.ipynb
In this [file](/create_ellipse/polyfit.ipynb) you can fit a polynomial to the points of the eyelid arc, found in create_ellipse.

## shift.ipynb
[File](/create_ellipse/shift.ipynb) for trying to understand pixel shift vs angular shift.
# coding: utf8

"""
Module de visualisation des résultats du rrtstar
on définit un fichier expérience sous le format suivant
==
= Coordinate system is cartesian with 0,0 at the lower left
==
--exp.rsc---
title
D w h
S x y //start coords
G x y w h //end coords
O x y w h //adds an obstacle (x,y) is bottom left, w is witdh, h heigth
O...
-----------

--res.rsc---
//to be more clearly defined
//same beginning as exp.rsc
//then
T xs ys xe ye // tree segment
T...
P xp yp // path intermediate point - expected in order
P...
"""
# import os

from solexp_classes import Solution, Experience


def main():
    """
    executed when file launched directly
    """
    # print(os.getcwd())
    # experience = loadExp("exp/exp.rsc")
    # print(experience)
    # dispExp(experience)
    sol = Solution()
    sol.configFromFile('exp/basicExp.sol')
    print(sol.exp)
    print(sol)
    sol.display(displayTree=True)


if __name__ == '__main__':
    main()

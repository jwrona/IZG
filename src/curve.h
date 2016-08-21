/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: curve.h 265 2013-02-20 16:44:40Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#ifndef Curve_H
#define Curve_H

/******************************************************************************
 * Includes
 */

#include "coords.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Definice funkci a maker pro praci s vektorem doublu
 */

USE_VECTOR_OF(double, dvec);

#define dvecGet(pVec, i)    (*dvecGetPtr((pVec), (i)))
#define S_DoubleVec         S_Vector


/******************************************************************************
 * Definice typu a fci pro praci s NURBS krivkou
 */

/* Struktura reprezentujici racionalni Bezierovu krivku ve 3D */
typedef struct S_Curve
{
    int         degree;            /* stupen krivky */
    S_CoordVec  * points;          /* vektor ridicich bodu krivky */
    S_DoubleVec * weights;         /* vektor vah ridicich bodu krivky */
    S_DoubleVec * knots;           /* uzlovy vektor */
} S_Curve;


/* Funkce vytvori novou strukturu popisujici NURBS krivku */
S_Curve * curveCreate();

/* Funkce nainicializuje krivku do "default" stavu, kdy
 * vahy ridicich bodu jsou nastavene na 1,
 * souradnice jsou nulove a knot vector je uniformni
 * size - pocet ridicich bodu krivky
 * k - stupen krivky */
void curveInit(S_Curve *pCurve, int size, int k);

/* Funkce nainicializuje uzlovy vektor krivky tak, aby byl uniformni
   Fce se automaticky vola uvnitr fce curveInit() */
void curveInitKnotVector(S_Curve *pCurve);

/* Fce vraci pocet ridicich bodu krivky */
int curveSize(S_Curve *pCurve);

/* Funkce korektne zrusi krivku a uvolni pamet */
void curveRelease(S_Curve **ppCurve);

/* Zmeni pocet ridicich bodu krivky a stupen krivky */
void curveResize(S_Curve *pCurve, int size, int k);

/* Spocita polohu bodu na krivce */
S_Coords curvePoint(S_Curve *pCurve, double t);


#ifdef __cplusplus
}
#endif

#endif // Curve_H

/*****************************************************************************/
/*****************************************************************************/

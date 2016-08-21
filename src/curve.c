/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: curve.c 265 2013-02-20 16:44:40Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#include "curve.h"

#include <stdlib.h>
#include <math.h>


/*****************************************************************************/

S_Curve * curveCreate()
{
    S_Curve * curve = (S_Curve *)malloc(sizeof(S_Curve));
    IZG_CHECK(curve, "Cannot allocate enough memory");

    curve->points = cvecCreateEmpty();
    curve->weights = dvecCreateEmpty();
    curve->knots = dvecCreateEmpty();
    curve->degree = 2;
    return curve;
}

/*****************************************************************************/

void curveInit(S_Curve *pCurve, int size, int k)
{
    IZG_ASSERT(pCurve && size >= 0 && k >= 2);

    /* stupen krivky */
    pCurve->degree = k;

    /* zmena velikosti vektoru */
    cvecResize(pCurve->points, size);
    dvecResize(pCurve->weights, size);

    /* inicializace uzloveho vektoru */
    curveInitKnotVector(pCurve);
}

/*****************************************************************************/

void curveInitKnotVector(S_Curve *pCurve)
{
    int         size, k, n, m, i;
    double      div, step;

    IZG_ASSERT(pCurve);

    /* parametry krivky, pocet ridicich bodu, apod. */
    size = cvecSize(pCurve->points);
    k = pCurve->degree;
    n = size - 1;
    m = n + k + 1;

    IZG_ASSERT(size >= 0 && k >= 2);

    /* zmena velikosti uzloveho vektoru */
    dvecResize(pCurve->knots, m + 1);

    /* uniformni uzlovy vektor */
    div = (double)(m - 2 * k);
    step = (div == 0.0) ? 0.0 : 1.0 / div;

    /* prvnich k+1 prvku vektoru je rovno 0 */
    for( i = 0; i < k + 1; ++i )
    {
        dvecGet(pCurve->knots, i) = 0.0;
    }
    /* zbyvajici cleny vektoru vyplnime konstantim krokem */
    for( i = k + 1; i < m - k; ++i )
    {
        dvecGet(pCurve->knots, i) = (i - k) * step;
    }
    /* poslednich k+1 prvku vektoru je rovno 1 */
    for( i = m - k; i < m + 1; ++i )
    {
        dvecGet(pCurve->knots, i) = 1.0;
    }
}

/*****************************************************************************/

int curveSize(S_Curve *pCurve)
{
    IZG_ASSERT(pCurve);

    return cvecSize(pCurve->points);
}

/*****************************************************************************/

void curveRelease(S_Curve **ppCurve)
{
    if( ppCurve && *ppCurve )
    {
        cvecRelease(&(*ppCurve)->points);
        dvecRelease(&(*ppCurve)->weights);

        free(*ppCurve);
        *ppCurve = NULL;
    }
}

/*****************************************************************************/

void curveResize(S_Curve *pCurve, int size, int k)
{
    int     i, n;

    IZG_ASSERT(pCurve && size >= 0 && k >= 2);

    /* stupen krivky */
    pCurve->degree = k;

    /* zmena velikosti vektoru */
    n = cvecSize(pCurve->points);
    cvecResize(pCurve->points, size);
    dvecResize(pCurve->weights, size);

    /* inicializace novych dat */
    for( i = n; i < size; ++i )
    {
        cvecGet(pCurve->points, i) = makeCoords(0.0, 0.0, 0.0);
        dvecGet(pCurve->weights, i) = 1.0;
    }

    /* inicializace uzloveho vektoru */
    curveInitKnotVector(pCurve);
}

/*****************************************************************************/

double splineFunc(int i, int k, S_DoubleVec * pKnots, double t)
{
    double      t1, t2, diff1, diff2;

    IZG_ASSERT( pKnots && i > -1 && i + k < dvecSize(pKnots) );

    if( k == 0 )
    {
        if( dvecGet(pKnots, i) <= t && t < dvecGet(pKnots, i + 1) ) return 1.0;
        else return 0.0;
    }
    
    diff1 = dvecGet(pKnots, i + k) - dvecGet(pKnots, i);
    diff2 = dvecGet(pKnots, i + k + 1) - dvecGet(pKnots, i + 1);

    t1 = IS_ZERO(diff1) ? 0.0 : (t - dvecGet(pKnots, i)) / diff1 * splineFunc(i, k - 1, pKnots, t);
    t2 = IS_ZERO(diff2) ? 0.0 : (dvecGet(pKnots, i + k + 1) - t) / diff2 * splineFunc(i + 1, k - 1, pKnots, t);
    
    return t1 + t2;
}

/*****************************************************************************/

S_Coords curvePoint(S_Curve *pCurve, double t)
{
    S_Coords    point = makeCoords(0.0, 0.0, 0.0);
    double      A, denom = 0.0;
    int         i, size;

    IZG_ASSERT(pCurve);

    /* velikost krivky */
    size = curveSize(pCurve);

    /* kontrola hodnoty parametru t <0, 1> */
    if( t < 0.0 ) t = 0.0;
    else if( t >= 1.0 ) t = 1.0;

    /* vypocet bodu krivky pomoci Bernsteinovych polynomu */
    for( i = 0; i < size; ++i )
    {
        A = dvecGet(pCurve->weights, i) * splineFunc(i, pCurve->degree, pCurve->knots, t);
        point.x += cvecGet(pCurve->points, i).x * A;
        point.y += cvecGet(pCurve->points, i).y * A;
        point.z += cvecGet(pCurve->points, i).z * A;
        denom += A;
    }     

    /* podeleni jmenovatelem */
    if( !IS_ZERO(denom) )
    {
        denom = 1.0 / denom;
    }
    point.x *= denom;
    point.y *= denom;
    point.z *= denom;

    return point;
}


/*****************************************************************************/
/*****************************************************************************/

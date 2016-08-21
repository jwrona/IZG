/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: model.h 48 2011-02-09 12:59:08Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#ifndef Model_H
#define Model_H

/******************************************************************************
 * Includes
 */

#include "coords.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Definice typu a fci pro praci s trojuhelniky
 */

/* Trojuhelnik ve 3D prostoru */
typedef struct S_Triangle
{
    int     v[3];   /* indexy vrcholu a soucasne normal ve vrcholech */
    int     n;      /* index normaly trojuhelniku */
} S_Triangle;


/* Vytvori novy trojuhelnik */
IZG_INLINE S_Triangle makeTriangle(int v0, int v1, int v2, int n)
{
    S_Triangle tri;
    tri.v[0] = v0;
    tri.v[1] = v1;
    tri.v[2] = v2;
    tri.n = n;
    return tri;
}


/******************************************************************************
 * Definice funkci a maker pro praci s vektorem trojuhelniku
 */

USE_VECTOR_OF(S_Triangle, trivec);

#define trivecGet(pVec, i)  (*trivecGetPtr((pVec), (i)))
#define S_TriVec            S_Vector


/******************************************************************************
 * Definice typu a fci pro praci s jednoduchym polygonalnim modelem.
 * - Representace polyg. modelu v pameti.
 * - Nacteni z textoveho souboru.
 */

/* Polygonalni model slozeny z trojuhelniku */
typedef struct S_Model
{
    S_CoordVec  * vertices;         /* vektor vsech vrcholu */
    S_CoordVec  * normals;          /* normaly ve vrcholech */
    S_TriVec    * triangles;        /* vektor trojuhelniku, ktere tvori model */
    S_CoordVec  * trinormals;       /* normaly techto trojuhelniku */
} S_Model;


/* Vytvori novou strukturu reprezentujici polyg. model */
S_Model * modCreate();

/* Nacte model ze souboru, v pripade chyby vraci funkce NULL */
S_Model * modLoad(const char * filename);

/* Zrusi model a uvolni veskerou pamet */
void modRelease(S_Model **ppModel);


#ifdef __cplusplus
}
#endif

#endif /* Model_H */

/*****************************************************************************/
/*****************************************************************************/

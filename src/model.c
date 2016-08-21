/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: model.c 48 2011-02-09 12:59:08Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#include "model.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


/*****************************************************************************/

S_Model * modCreate()
{
    S_Model * model = (S_Model *)malloc(sizeof(S_Model));
    IZG_CHECK(model, "Cannot allocate enough memory");

    model->vertices = cvecCreateEmpty();
    model->normals = cvecCreateEmpty();
    model->triangles = trivecCreateEmpty();
    model->trinormals = cvecCreateEmpty();
    return model;
}

/*****************************************************************************/

S_Model * modLoad(const char * filename)
{
    FILE        * f;
    int         n, t, i, v0, v1, v2;
    S_Model     * model;
    S_Coords    * vertex, * normal;
    S_Coords    a, b, c;

    /* otevri vstupni soubor */
    if( (f = fopen(filename, "r")) == NULL )
    {
        return NULL;
    }

    /* nacteni hlavicky */
    if( fscanf(f, "tri %d.%d\n", &v0, &v1) != 2 )
    {
        return NULL;
    }

    /* nacteni poctu vrcholu */
    if( fscanf(f, "vertices %d\n", &n) != 1 || n < 0 )
    {
        return NULL;
    }

    /* nacteni poctu trojuhelniku */
    if( fscanf(f, "faces %d\n", &t) != 1 || t < 0 )
    {
        return NULL;
    }

    /* vytvor novy model */
    model = modCreate();
    cvecResize(model->vertices, n);
    cvecResize(model->normals, n);
    trivecResize(model->triangles, t);
    cvecResize(model->trinormals, t);

    /* nacteni vrcholu a soucasne vymazani vsech normal ve vrcholech */
    for( i = 0; i < n; ++i )
    {
        vertex = cvecGetPtr(model->vertices, i);
        if( fscanf(f, "%lf %lf %lf\n", &vertex->x, &vertex->y, &vertex->z) != 3 )
        {
            modRelease(&model);
            return NULL;
        }
        
        /* vymazani odpovidajici normaly */
        normal = cvecGetPtr(model->normals, i);
        normal->x = 0;
        normal->y = 0;
        normal->z = 0;
    }

    /* nacteni trojuhelniku a akumulace jejich vypoctene normaly ve vrcholech */
    for( i = 0; i < t; ++i )
    {
        if( fscanf(f, "%d %d %d\n", &v0, &v1, &v2) != 3 
            || v0 < 0 || v0 >= n
            || v1 < 0 || v1 >= n
            || v2 < 0 || v2 >= n )
        {
            modRelease(&model);
            return NULL;
        }

        /* vektor v0 -> v1 */
        a.x = cvecGet(model->vertices, v1).x - cvecGet(model->vertices, v0).x;
        a.y = cvecGet(model->vertices, v1).y - cvecGet(model->vertices, v0).y;
        a.z = cvecGet(model->vertices, v1).z - cvecGet(model->vertices, v0).z;

        /* vektor v1 -> v2 */
        b.x = cvecGet(model->vertices, v2).x - cvecGet(model->vertices, v0).x;
        b.y = cvecGet(model->vertices, v2).y - cvecGet(model->vertices, v0).y;
        b.z = cvecGet(model->vertices, v2).z - cvecGet(model->vertices, v0).z;

        /* vypocet normaly trojuhelniku */
        c.x = a.y * b.z - a.z * b.y;
        c.y = a.z * b.x - a.x * b.z;
        c.z = a.x * b.y - a.y * b.x;
        coordsNormalize(&c);

        /* ulozeni normaly */
        cvecGet(model->trinormals, i) = c;

        /* akumulace normaly ve vrcholech trojuhelniku (~interpolace normal ve vrcholech) */
        normal = cvecGetPtr(model->normals, v0);
        normal->x += c.x;
        normal->y += c.y;
        normal->z += c.z;

        normal = cvecGetPtr(model->normals, v1);
        normal->x += c.x;
        normal->y += c.y;
        normal->z += c.z;

        normal = cvecGetPtr(model->normals, v2);
        normal->x += c.x;
        normal->y += c.y;
        normal->z += c.z;

        /* novy trojuhelnik */
        trivecGet(model->triangles, i) = makeTriangle(v0, v1, v2, i);
    }

    /* normalizace normal */
    for( i = 0; i < n; ++i )
    {       
        normal = cvecGetPtr(model->normals, i);
        coordsNormalize(normal);
    }

    /* vse nacteno v poradku... */
    return model;
}

/*****************************************************************************/

void modRelease(S_Model **ppModel)
{
    if( ppModel && *ppModel )
    {
        cvecRelease(&(*ppModel)->vertices);
        cvecRelease(&(*ppModel)->normals);
        trivecRelease(&(*ppModel)->triangles);
        cvecRelease(&(*ppModel)->trinormals);

        free(*ppModel);
        *ppModel = NULL;
    }
}


/*****************************************************************************/
/*****************************************************************************/

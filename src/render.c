/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: render.c 264 2013-02-20 12:27:25Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#include "render.h"
#include "transform.h"

#include <memory.h>


/*****************************************************************************/

S_Renderer * renCreate()
{
    S_Renderer * renderer = (S_Renderer *)malloc(sizeof(S_Renderer));
    IZG_CHECK(renderer, "Cannot allocate enough memory");

    renInit(renderer);
    return renderer;
}

/*****************************************************************************/

void renInit(S_Renderer *pRenderer)
{
    IZG_ASSERT(pRenderer);

    /* frame buffer prozatim neni */
    pRenderer->frame_buffer = NULL;
    pRenderer->frame_w = 0;
    pRenderer->frame_h = 0;

    /* podobne take depth buffer */
    pRenderer->depth_buffer = NULL;
    pRenderer->max_depth = 1000.0;

    /* nastaveni pozice kamery */
    pRenderer->camera_dist = 1000;

    /* pocatecni nastaveni trackballu (natoceni a zoom sceny) */
    pRenderer->scene_rot_x = 0;
    pRenderer->scene_rot_y = 0;
    pRenderer->scene_move_z = -980;
    pRenderer->scene_move_x = 0;
    pRenderer->scene_move_y = 0;

    /* default material */
    pRenderer->mat_ambient = makeMaterial(0.8, 0.8, 0.8);
    pRenderer->mat_diffuse = makeMaterial(0.8, 0.8, 0.8);
    pRenderer->mat_specular = makeMaterial(0.8, 0.8, 0.8);

    /* default zdroj svetla */
    pRenderer->light_position  = makeCoords(3.0, -3.0, -pRenderer->camera_dist);
    pRenderer->light_ambient   = makeLight(0.2, 0.2, 0.2);
    pRenderer->light_diffuse   = makeLight(0.7, 0.7, 0.7);
    pRenderer->light_specular  = makeLight(0.7, 0.7, 0.7);

    /* nastaveni ukazatelu na fce */
    pRenderer->releaseFunc         = renRelease;
    pRenderer->createBuffersFunc   = renCreateBuffers;
    pRenderer->clearBuffersFunc    = renClearBuffers;
    pRenderer->projectTriangleFunc = renProjectTriangle;
    pRenderer->projectLineFunc     = renProjectLine;
    pRenderer->calcReflectanceFunc = renLambertianReflectance;
}

/*****************************************************************************/

void renRelease(S_Renderer **ppRenderer)
{
    if( ppRenderer && *ppRenderer )
    {
        /* uvolneni pameti frame bufferu */
        if( (*ppRenderer)->frame_buffer )
        {
            free((*ppRenderer)->frame_buffer);
        }

        /* uvolneni pameti depth bufferu */
        if( (*ppRenderer)->depth_buffer )
        {
            free((*ppRenderer)->depth_buffer);
        }

        free(*ppRenderer);
        *ppRenderer = NULL;
    }
}

/*****************************************************************************/

void renCreateBuffers(S_Renderer *pRenderer, int width, int height)
{
    S_RGBA  * new_frame_buffer;
    double  * new_depth_buffer;

    IZG_ASSERT(pRenderer && width > 0 && height > 0);

    /* alokace pameti frame bufferu */
    new_frame_buffer = (S_RGBA *)realloc(pRenderer->frame_buffer, width * height * sizeof(S_RGBA));
    IZG_CHECK(new_frame_buffer, "Cannot allocate frame buffer");

    /* vymazani obsahu frame bufferu */
    memset(new_frame_buffer, 0, width * height * sizeof(S_RGBA));

    /* nastaveni nove velikosti frame bufferu */
    pRenderer->frame_buffer = new_frame_buffer;
    pRenderer->frame_w = width;
    pRenderer->frame_h = height;

    /* alokace pameti depth bufferu */
    new_depth_buffer = (double *)realloc(pRenderer->depth_buffer, width * height * sizeof(double));
    IZG_CHECK(new_depth_buffer, "Cannot allocate depth buffer");

    /* vymazani obsahu depth bufferu */
    memset(new_depth_buffer, 0, width * height * sizeof(double));

    /* nastaveni ukazatele */
    pRenderer->depth_buffer = new_depth_buffer;
}

/*****************************************************************************/

void renClearBuffers(S_Renderer *pRenderer)
{
    int     i, count;

    IZG_ASSERT(pRenderer);

    /* vsechny pixely cerne, tedy 0 */
    memset(pRenderer->frame_buffer, 0, pRenderer->frame_w * pRenderer->frame_h * sizeof(S_RGBA));

    /* vymazeme depth buffer */
    count = pRenderer->frame_w * pRenderer->frame_h;
    for( i = 0; i < count; ++i )
    {
        *(pRenderer->depth_buffer + i) = pRenderer->max_depth;
    }
}

/*****************************************************************************/
/*****************************************************************************/

void renMatAmbient(S_Renderer *pRenderer, const S_Material *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->mat_ambient = *pParams;
}

/*****************************************************************************/

void renMatDiffuse(S_Renderer *pRenderer, const S_Material *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->mat_diffuse = *pParams;
}

/*****************************************************************************/

void renMatSpecular(S_Renderer *pRenderer, const S_Material *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->mat_specular = *pParams;
}

/*****************************************************************************/

void renLightPosition(S_Renderer *pRenderer, const S_Coords *pPosition)
{
    IZG_ASSERT(pRenderer && pPosition);

    pRenderer->light_position = *pPosition;
}

/*****************************************************************************/

void renLightAmbient(S_Renderer *pRenderer, const S_Light *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->light_ambient = *pParams;
}

/*****************************************************************************/

void renLightDiffuse(S_Renderer *pRenderer, const S_Light *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->light_diffuse = *pParams;
}

/*****************************************************************************/

void renLightSpecular(S_Renderer *pRenderer, const S_Light *pParams)
{
    IZG_ASSERT(pRenderer && pParams);

    pRenderer->light_specular = *pParams;
}

/*****************************************************************************/
/*****************************************************************************/

void renSceneRotXY(S_Renderer *pRenderer, int dx, int dy)
{
    IZG_ASSERT(pRenderer);

    pRenderer->scene_rot_x += 0.01 * dx;
    pRenderer->scene_rot_y += 0.01 * dy;
}

/*****************************************************************************/

void renSceneMoveZ(S_Renderer *pRenderer, int dz)
{
    IZG_ASSERT(pRenderer);

    pRenderer->scene_move_z += 0.5 * dz;

    /* nejsme uz prilis blizko ke kamere? */
    if( pRenderer->scene_move_z < (-pRenderer->camera_dist + 3) )
    {
        pRenderer->scene_move_z = -pRenderer->camera_dist + 3;
    }
}

/*****************************************************************************/

void renSceneMoveXY(S_Renderer *pRenderer, int dx, int dy)
{
    IZG_ASSERT(pRenderer);

    pRenderer->scene_move_x += 0.1 * dx;
    pRenderer->scene_move_y += 0.1 * dy;
}

/*****************************************************************************/

void renSetupTrackball(S_Renderer *pRenderer)
{
    IZG_ASSERT(pRenderer);

    /* nejprve nastavime posuv cele sceny od/ke kamere */
    trTranslate(0.0, 0.0, pRenderer->scene_move_z);

    /* natoceni cele sceny - jen ve dvou smerech - mys je jen 2D... :( */
    trRotateX(pRenderer->scene_rot_x);
    trRotateY(pRenderer->scene_rot_y);
}

/*****************************************************************************/

void renSetupProjection(S_Renderer *pRenderer)
{
    IZG_ASSERT(pRenderer);

    trProjectionPerspective(pRenderer->camera_dist, pRenderer->frame_w, pRenderer->frame_h);
}

/*****************************************************************************/
/*****************************************************************************/

S_RGBA renLambertianReflectance(S_Renderer *pRenderer, const S_Coords *point, const S_Coords *normal)
{
    S_Coords    lvec;
    double      diffuse, r, g, b;
    S_RGBA      color;

    IZG_ASSERT(pRenderer && point && normal);

    /* vektor ke zdroji svetla */
    lvec = makeCoords(pRenderer->light_position.x - point->x,
                      pRenderer->light_position.y - point->y,
                      pRenderer->light_position.z - point->z);
    coordsNormalize(&lvec);

    /* ambientni cast */
    r = pRenderer->light_ambient.red * pRenderer->mat_ambient.red;
    g = pRenderer->light_ambient.green * pRenderer->mat_ambient.green;
    b = pRenderer->light_ambient.blue * pRenderer->mat_ambient.blue;

    /* difuzni cast */
    diffuse = lvec.x * normal->x + lvec.y * normal->y + lvec.z * normal->z;
    if( diffuse > 0 )
    {
        r += diffuse * pRenderer->light_diffuse.red * pRenderer->mat_diffuse.red;
        g += diffuse * pRenderer->light_diffuse.green * pRenderer->mat_diffuse.green;
        b += diffuse * pRenderer->light_diffuse.blue * pRenderer->mat_diffuse.blue;
    }
    
    /* saturace osvetleni*/
    r = MIN(1, r);
    g = MIN(1, g);
    b = MIN(1, b);

    /* kreslici barva */
    color.red = ROUND2BYTE(255 * r);
    color.green = ROUND2BYTE(255 * g);
    color.blue = ROUND2BYTE(255 * b);
    return color;
}

/*****************************************************************************/

int renCalcVisibility(S_Renderer *pRenderer, const S_Coords *point, const S_Coords *normal)
{
    S_Coords cvec;

    IZG_ASSERT(pRenderer && point && normal);

    /* vektor od kamery k plosce, pozice kamery je (0, 0, -camera_dist) */
    cvec = makeCoords(point->x, point->y, point->z + pRenderer->camera_dist);
    coordsNormalize(&cvec);

    /* test zda je normala privracena
     * skalarni soucin vektoru od kamery a normaly */
    return (normal->x * cvec.x + normal->y * cvec.y + normal->z * cvec.z > 0) ? 0 : 1;
}

/****************************************************************************
 ****************************************************************************
 * Orezani usecky algoritmem Liang-Barsky pred vykreslenim do frame bufferu
 * Vraci -1 je-li usecka zcela mimo okno
 * x1, y1, x2, y2 - koncove body usecky */

int renCullLine(S_Renderer *pRenderer, int *x1, int *y1, int *x2, int *y2)
{
    int     dx, dy, p[4], q[4], i;
    float   u1, u2, t1, t2, ri;

    IZG_ASSERT(x1 && y1 && x2 && y2);

    /* vypocet parametru dx, dy */
    dx = *x2 - *x1;
    dy = *y2 - *y1;

    /* vypocet parametru pi a qi */
    p[0] = -dx; q[0] = *x1 - 1;
    p[1] =  dx; q[1] = pRenderer->frame_w - 2 - *x1;
    p[2] = -dy; q[2] = *y1 - 1;
    p[3] =  dy; q[3] = pRenderer->frame_h - 2 - *y1;

    /* nastaveni parametru ui na pocatecni hodnotu */
    u1 = 0.0f;
    u2 = 1.0f;

    /* postupne proverime pi a qi */
    for( i = 0; i < 4; ++i )
    {
        /* vypocet parametru ri */
        ri = 0.0f;
        if( p[i] != 0.0f )
        {
            ri = (float)q[i] / (float)p[i];
        }

        /* primka smeruje do okna */
        if( p[i] < 0 )
        {
            if( ri > u2 ) return -1;
            else if( ri > u1 ) u1 = ri;
        }
        else if( p[i] > 0 )     /* primka smeruje z okna ven */
        {
            if( ri < u1 ) return -1;
            else if( ri < u2 ) u2 = ri;
            
        }
        else if( q[i] < 0 )     /* primka je mimo okno */
        {
            return -1;
        }
    }

    /* spocteme vysledne souradnice x1,y1 a x2,y2 primky */
    if( u2 < 1 )
    {
        t1 = u2 * dx;
        t2 = u2 * dy;
        *x2 = *x1 + ROUND(t1);
        *y2 = *y1 + ROUND(t2);
    }
    if( u1 > 0 )
    {
        t1 = u1 * dx;
        t2 = u1 * dy;
        *x1 = *x1 + ROUND(t1);
        *y1 = *y1 + ROUND(t2);
    }

    return 0;
}

/****************************************************************************
 * Rasterizace usecky do frame bufferu pomoci Bresenhamova algoritmu
 * x1, y1, x2, y2 - souradnice koncovych bodu ve 2D po projekci
 * color - kreslici barva */

void renDrawLine(S_Renderer *pRenderer,
                 int x1, int y1,
                 int x2, int y2,
                 S_RGBA color
                 )
{
    int     dx, dy, xy_swap, step_y, k1, k2, p, x, y;

    /* orezani do okna */
    if( renCullLine(pRenderer, &x1, &y1, &x2, &y2) == -1 )
    {
        return;
    }

    /* delta v ose x a y */
    dx = ABS(x2 - x1);
    dy = ABS(y2 - y1);

    /* zamena osy x a y v algoritmu vykreslovani */
    xy_swap = 0;
    if( dy > dx )
    {
        xy_swap = 1;
        SWAP(x1, y1);
        SWAP(x2, y2);
        SWAP(dx, dy);
    }

    /* pripadne prohozeni bodu v ose x */
    if( x1 > x2 )
    {
        SWAP(x1, x2);
        SWAP(y1, y2);
    }

    /* smer postupu v ose y */
    if( y1 > y2 ) step_y = -1;
    else step_y = 1;

    /* konstanty pro Bresenhamuv algoritmus */
    k1 = 2 * dy;
    k2 = 2 * (dy - dx);
    p  = 2 * dy - dx;

    /* pocatecni prirazeni hodnot */
    x = x1;
    y = y1;

    /* cyklus vykreslovani usecky */
    for( ; x <= x2; ++x )
    {
        if( xy_swap ) PIXEL(pRenderer, y, x) = color;
        else PIXEL(pRenderer, x, y) = color;
        
        /* uprava prediktoru */
        if( p < 0 )
        {
            p += k1;
        }
        else
        {
            p += k2;
            y += step_y;
        }
    }
}

/*****************************************************************************/

void renProjectLine(S_Renderer *pRenderer, S_Coords *p1, S_Coords *p2, S_RGBA color)
{
    S_Coords    aa, bb;             /* souradnice vrcholu po transformaci ve 3D pred projekci */
    int         u1, v1, u2, v2;     /* souradnice vrcholu po projekci do roviny obrazovky */

    IZG_ASSERT(pRenderer && p1 && p2);

    /* transformace vrcholu matici model */
    trTransformVertex(&aa, p1);
    trTransformVertex(&bb, p2);

    /* promitneme vrcholy na obrazovku */
    trProjectVertex(&u1, &v1, &aa);
    trProjectVertex(&u2, &v2, &bb);
    
    /* rasterizace usecky */
    renDrawLine(pRenderer, u1, v1, u2, v2, color);
}


/****************************************************************************
 ****************************************************************************
 * Rasterizace trojuhelniku danych souradnic a barvy do frame bufferu
 * vcetne interpolace z souradnice a prace se z-bufferem
 * a interpolaci barvy (tzv. Goraudovo stinovani)
 * v1, v2, v3 - ukazatele na vrcholy trojuhelniku ve 3D pred projekci
 * n1, n2, n3 - ukazatele na normaly ve vrcholech ve 3D pred projekci
 * x1, y1, ... - vrcholy trojuhelniku po projekci do roviny obrazovky */

void renDrawTriangle(S_Renderer *pRenderer,
                     S_Coords *v1, S_Coords *v2, S_Coords *v3,
                     S_Coords *n1, S_Coords *n2, S_Coords *n3,
                     int x1, int y1,
                     int x2, int y2,
                     int x3, int y3
                     )
{
    int         minx, miny, maxx, maxy;
    int         a1, a2, a3, b1, b2, b3, c1, c2, c3;
    int         s1, s2, s3;
    int         x, y, e1, e2, e3;
    double      alpha, beta, gamma, w1, w2, w3, z;
    S_RGBA      col1, col2, col3, color;

    IZG_ASSERT(pRenderer && v1 && v2 && v3 && n1 && n2 && n3);

    /* vypocet barev ve vrcholech */
    col1 = pRenderer->calcReflectanceFunc(pRenderer, v1, n1);
    col2 = pRenderer->calcReflectanceFunc(pRenderer, v2, n2);
    col3 = pRenderer->calcReflectanceFunc(pRenderer, v3, n3);

    /* obalka trojuhleniku */
    minx = MIN(x1, MIN(x2, x3));
    maxx = MAX(x1, MAX(x2, x3));
    miny = MIN(y1, MIN(y2, y3));
    maxy = MAX(y1, MAX(y2, y3));

    /* oriznuti podle rozmeru okna */
    miny = MAX(miny, 0);
    maxy = MIN(maxy, pRenderer->frame_h - 1);
    minx = MAX(minx, 0);
    maxx = MIN(maxx, pRenderer->frame_w - 1);

    /* Pineduv alg. rasterizace troj.
       hranova fce je obecna rovnice primky Ax + By + C = 0
       primku prochazejici body (x1, y1) a (x2, y2) urcime jako
       (y1 - y2)x + (x2 - x1)y + x1y2 - x2y1 = 0 */

    /* normala primek - vektor kolmy k vektoru mezi dvema vrcholy, tedy (-dy, dx) */
    a1 = y1 - y2;
    a2 = y2 - y3;
    a3 = y3 - y1;
    b1 = x2 - x1;
    b2 = x3 - x2;
    b3 = x1 - x3;

    /* koeficient C */
    c1 = x1 * y2 - x2 * y1;
    c2 = x2 * y3 - x3 * y2;
    c3 = x3 * y1 - x1 * y3;

    /* vypocet hranove fce (vzdalenost od primky) pro protejsi body */
    s1 = a1 * x3 + b1 * y3 + c1;
    s2 = a2 * x1 + b2 * y1 + c2;
    s3 = a3 * x2 + b3 * y2 + c3;

    /* normalizace, aby vzdalenost od primky byla kladna uvnitr trojuhelniku */
    if( s1 < 0 )
    {
        a1 *= -1;
        b1 *= -1;
        c1 *= -1;
    }
    if( s2 < 0 )
    {
        a2 *= -1;
        b2 *= -1;
        c2 *= -1;
    }
    if( s3 < 0 )
    {
        a3 *= -1;
        b3 *= -1;
        c3 *= -1;
    }

    /* koeficienty pro barycentricke souradnice */
    alpha = 1.0 / ABS(s2);
    beta = 1.0 / ABS(s3);
    gamma = 1.0 / ABS(s1);

    /* vyplnovani... */
    for( y = miny; y <= maxy; ++y )
    {
        /* inicilizace hranove fce v bode (minx, y) */
        e1 = a1 * minx + b1 * y + c1;
        e2 = a2 * minx + b2 * y + c2;
        e3 = a3 * minx + b3 * y + c3;

        for( x = minx; x <= maxx; ++x )
        {
            if( e1 >= 0 && e2 >= 0 && e3 >= 0 )
            {
                /* interpolace pomoci barycentrickych souradnic
                   e1, e2, e3 je aktualni vzdalenost bodu (x, y) od primek */
                w1 = alpha * e2;
                w2 = beta * e3;
                w3 = gamma * e1;

                /* interpolace z-souradnice */
                z = w1 * v1->z + w2 * v2->z + w3 * v3->z;
               
                /* interpolace barvy */
                color.red = ROUND2BYTE(w1 * col1.red + w2 * col2.red + w3 * col3.red);
                color.green = ROUND2BYTE(w1 * col1.green + w2 * col2.green + w3 * col3.green);
                color.blue = ROUND2BYTE(w1 * col1.blue + w2 * col2.blue + w3 * col3.blue);
                color.alpha = 255;

                /* vykresleni bodu */
                if( z < DEPTH(pRenderer, x, y) )
                {
                    PIXEL(pRenderer, x, y) = color;
                    DEPTH(pRenderer, x, y) = z;
                }
            }

            /* hranova fce o pixel vedle */
            e1 += a1;
            e2 += a2;
            e3 += a3;
        }
    }
}

/*****************************************************************************/

void renProjectTriangle(S_Renderer *pRenderer, S_Model *pModel, int i)
{
    S_Coords    aa, bb, cc;             /* souradnice vrcholu po transformaci */
    S_Coords    naa, nbb, ncc;          /* normaly ve vrcholech po transformaci */
    S_Coords    nn;                     /* normala trojuhelniku po transformaci */
    int         u1, v1, u2, v2, u3, v3; /* souradnice vrcholu po projekci do roviny obrazovky */
    S_Triangle  * triangle;

    IZG_ASSERT(pRenderer && pModel && i >= 0 && i < trivecSize(pModel->triangles));

    /* z modelu si vytahneme trojuhelnik */
    triangle = trivecGetPtr(pModel->triangles, i);

    /* transformace vrcholu matici model */
    trTransformVertex(&aa, cvecGetPtr(pModel->vertices, triangle->v[0]));
    trTransformVertex(&bb, cvecGetPtr(pModel->vertices, triangle->v[1]));
    trTransformVertex(&cc, cvecGetPtr(pModel->vertices, triangle->v[2]));

    /* promitneme vrcholy trojuhelniku na obrazovku */
    trProjectVertex(&u1, &v1, &aa);
    trProjectVertex(&u2, &v2, &bb);
    trProjectVertex(&u3, &v3, &cc);

    /* pro osvetlovaci model transformujeme take normaly ve vrcholech */
    trTransformVector(&naa, cvecGetPtr(pModel->normals, triangle->v[0]));
    trTransformVector(&nbb, cvecGetPtr(pModel->normals, triangle->v[1]));
    trTransformVector(&ncc, cvecGetPtr(pModel->normals, triangle->v[2]));

    /* normalizace normal */
    coordsNormalize(&naa);
    coordsNormalize(&nbb);
    coordsNormalize(&ncc);

    /* transformace normaly trojuhelniku matici model */
    trTransformVector(&nn, cvecGetPtr(pModel->trinormals, triangle->n));
    
    /* normalizace normaly */
    coordsNormalize(&nn);

    /* je troj. privraceny ke kamere, tudiz viditelny? */
    if( !renCalcVisibility(pRenderer, &aa, &nn) )
    {
        /* odvracene troj. vubec nekreslime */
        return;
    }

    /* rasterizace trojuhelniku */
    renDrawTriangle(pRenderer,
                    &aa, &bb, &cc,
                    &naa, &nbb, &ncc,
                    u1, v1, u2, v2, u3, v3
                    );
}


/*****************************************************************************/
/*****************************************************************************/

/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: student.c 265 2013-02-20 16:44:40Z spanel $
 */

#include "student.h"
#include "transform.h"
#include "curve.h"

#include <memory.h>
#include <math.h>


/******************************************************************************
 * Globalni promenne a konstanty
 */

/* krivka predstavujici drahu telesa */
S_Curve             * trajectory    = NULL;

/* pocet kroku pri vykreslovani krivky */
const int           CURVE_QUALITY   = 100;

/* dalsi globalni promenne a konstanty */
double rotation1 = 0.0, rotation2 = 3.1415;
/* ??? */


/*****************************************************************************
 * Funkce vytvori vas renderer a nainicializuje jej */

S_Renderer * studrenCreate()
{
    S_StudentRenderer * renderer = (S_StudentRenderer *)malloc(sizeof(S_StudentRenderer));
    IZG_CHECK(renderer, "Cannot allocate enough memory");

    /* inicializace default rendereru */
    renInit(&renderer->base);

    /* nastaveni ukazatelu na vase upravene funkce */
    renderer->base.projectLineFunc = studrenProjectLine;

    /* inicializace pripadnych nove pridanych casti */
    /* ??? */

    return (S_Renderer *)renderer;
}

/****************************************************************************
 * Orezani usecky algoritmem Liang-Barsky pred vykreslenim do frame bufferu
 * Vraci -1 je-li usecka zcela mimo okno
 * x1, y1, x2, y2 - puvodni a pozdeji orezane koncove body usecky 
 * u1, u2 - hodnoty parametru u pro orezanou cast usecky <0, 1> */

int studrenCullLine(S_Renderer *pRenderer,
                    int *x1, int *y1, int *x2, int *y2,
                    double *u1, double *u2
                    )
{
    //zmena u1, u2 na *u1, *u2 oproti renCullLine()

    int     dx, dy, p[4], q[4], i;
    float   t1, t2, ri;

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
    *u1 = 0.0f;
    *u2 = 1.0f;

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
            if( ri > *u2 ) return -1;
            else if( ri > *u1 ) *u1 = ri;
        }
        else if( p[i] > 0 )     /* primka smeruje z okna ven */
        {
            if( ri < *u1 ) return -1;
            else if( ri < *u2 ) *u2 = ri;
            
        }
        else if( q[i] < 0 )     /* primka je mimo okno */
        {
            return -1;
        }
    }

    /* spocteme vysledne souradnice x1,y1 a x2,y2 primky */
    if( *u2 < 1 )
    {
        t1 = *u2 * dx;
        t2 = *u2 * dy;
        *x2 = *x1 + ROUND(t1);
        *y2 = *y1 + ROUND(t2);
    }
    if( *u1 > 0 )
    {
        t1 = *u1 * dx;
        t2 = *u1 * dy;
        *x1 = *x1 + ROUND(t1);
        *y1 = *y1 + ROUND(t2);
    }

    return 0;
}

/****************************************************************************
 * Rasterizace usecky do frame bufferu s interpolaci z-souradnice
 * a zapisem do z-bufferu
 * v1, v2 - ukazatele na kocove body usecky ve 3D prostoru pred projekci
 * x1, y1, x2, y2 - souradnice koncovych bodu ve 2D po projekci
 * color - kreslici barva */

void studrenDrawLine(S_Renderer *pRenderer,
                     S_Coords *v1, S_Coords *v2,
                     int x1, int y1,
                     int x2, int y2,
                     S_RGBA color
                     )
{
    int     dx, dy, xy_swap, step_y, k1, k2, p, x, y;
    double u1, u2, dz, depth;

    /* orezani do okna */
    if(studrenCullLine(pRenderer, &x1, &y1, &x2, &y2, &u1, &u2)== -1)
        return;

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

    /* MOJE - z krok */
    //z2 - z1 = delka usecky v prostoru
    //dx = delka usecky v ose x (pocet vykreslenych pixelu usecky)
    //jejich podil vytvori krok, o ktery se zmeni
    //z souradnice (hloubka) pri zmene jednoho pixelu v ose x
    dz = (v2->z - v1->z) / dx;

    /* cyklus vykreslovani usecky */
    for(depth = v1->z; x <= x2; ++x)
    {
        if(xy_swap){
	    if (depth < DEPTH(pRenderer, y, x)){ //pokud je hloubka aktualniho pixelu mensi, 
	    //nez hloubka na prislusne pozici v z-bufferu, tak je blize ke kamere a tudiz
	    //prekryva objekt, ktery je dale
	        PIXEL(pRenderer, y, x) = color; //vykresli pixel
		DEPTH(pRenderer, y, x) = depth; //uloz hloubku do z-bufferu
	    }
	}
        else {
	    if (depth < DEPTH(pRenderer, x, y)){ // to stejne jako predtim
	        PIXEL(pRenderer, x, y) = color;
		DEPTH(pRenderer, x, y) = depth;
	    }
	}
        
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

	depth += dz; //pricteni z-kroku k aktualni hloubce pixelu
    }

}


/******************************************************************************
 * Vykresli caru zadanou barvou
 * Pred vykreslenim aplikuje na koncove body cary aktualne nastavene
 * transformacni matice!
 * p1, p2 - koncove body
 * color - kreslici barva */

void studrenProjectLine(S_Renderer *pRenderer, S_Coords *p1, S_Coords *p2, S_RGBA color)
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
    studrenDrawLine(pRenderer, &aa, &bb, u1, v1, u2, v2, color);
}

/******************************************************************************
 * Funkce pro vykresleni krivky jako "lomene cary"
 * color - kreslici barva
 * n - pocet bodu na krivce, kolik se ma pouzit pro aproximaci */

void renderCurve(S_Renderer *pRenderer, S_Curve *pCurve, S_RGBA color, int n)
{
    double step = 1.0 / n; //krok
    double point; //aktualni bod
    S_Coords from, to; //souradnice odkud pokud se bude vykreslovat

    for (point = 0.0; point <= 1.0; ){
        from = curvePoint(pCurve, point); //vrati bod na krivce, point musi byt v <0, 1>
        point += step; //pricti krok
        to = curvePoint(pCurve, point); 

        //vykresleni jedne usecky
        studrenProjectLine(pRenderer, &from, &to, color);

	//pro pristi iteraci priradit soucasny koncovy bod
	//do pocatecniho bodu
	from = to;
    }

    //vykresleni posledni usecky
    to = curvePoint(pCurve, 1.0);
    studrenProjectLine(pRenderer, &from, &to, color);
}

/******************************************************************************
 * Callback funkce volana pri startu aplikace */

void onInit()
{
    /* vytvoreni a inicializace krivky */
    trajectory = curveCreate();
    curveInit(trajectory, 7, 2);

    //zadani Ut
    dvecGet(trajectory->knots, 0) = 0;
    dvecGet(trajectory->knots, 1) = 0;
    dvecGet(trajectory->knots, 2) = 0;
    dvecGet(trajectory->knots, 3) = 0.25;
    dvecGet(trajectory->knots, 4) = 0.5;
    dvecGet(trajectory->knots, 5) = 0.5;
    dvecGet(trajectory->knots, 6) = 0.75;
    dvecGet(trajectory->knots, 7) = 1;
    dvecGet(trajectory->knots, 8) = 1;
    dvecGet(trajectory->knots, 9) = 1;

    //zadani uzlovych bodu a jejich vah
    cvecGet(trajectory->points, 0) = makeCoords(0.0, 0.0, 0.0);
    dvecGet(trajectory->weights, 0) = 1.0;

    cvecGet(trajectory->points, 1) = makeCoords(0.0, 0.0, -1.0);
    dvecGet(trajectory->weights, 1) = 0.5;

    cvecGet(trajectory->points, 2) = makeCoords(2.0, 0.0, -1.0);
    dvecGet(trajectory->weights, 2) = 0.5;

    cvecGet(trajectory->points, 3) = makeCoords(2.0, 0.0, 0.0);
    dvecGet(trajectory->weights, 3) = 1.0;

    cvecGet(trajectory->points, 4) = makeCoords(2.0, 0.0, 1.0);
    dvecGet(trajectory->weights, 4) = 0.5;

    cvecGet(trajectory->points, 5) = makeCoords(0.0, 0.0, 1.0);
    dvecGet(trajectory->weights, 5) = 0.5;

    cvecGet(trajectory->points, 6) = makeCoords(0.0, 0.0, 0.0);
    dvecGet(trajectory->weights, 6) = 1.0;

}

/******************************************************************************
 * Callback funkce volana pri tiknuti casovace */

void onTimer(int ticks)
{
    /* uprava pozice planetky */
    rotation1 += 0.05; //pro model na mensi kruznici
    rotation2 += 0.02; //pro model na vetsi kruznici
    
}

/******************************************************************************
 * Funkce pro vyrenderovani sceny, tj. vykresleni modelu
 * Upravte pro kresleni dynamicke sceny, ve ktere se "planetky"
 * pohybuji po krivce, tj. mirne elipticke draze. */

void renderScene(S_Renderer *pRenderer, S_Model *pModel)
{
    const double circle1Size = 3.0, model1Size = 0.3; //velikost kruznic a modelu
    const double circle2Size = 5.5, model2Size = 0.5;
    S_Matrix savedMatrix; //zde bude ulozena referencni matice

    const S_Material BLUE_AMBIENT = {0.2, 0.2, 0.8}; //barvy a materialy
    const S_Material BLUE_DIFFUSE = {0.2, 0.2, 0.8};
    const S_Material BLUE_SPECULAR = {0.8, 0.8, 0.8};
    
    const S_Material GREEN_AMBIENT = {0.2, 0.8, 0.2};
    const S_Material GREEN_DIFFUSE = {0.2, 0.8, 0.2};
    const S_Material GREEN_SPECULAR = {0.8, 0.8, 0.8};

    const S_RGBA COLOR_GREEN = {0, 255, 0, 255};

    /* test existence frame bufferu a modelu */
    IZG_ASSERT(pModel && pRenderer);

    /* nastavit projekcni matici */
    trProjectionPerspective(pRenderer->camera_dist, pRenderer->frame_w, pRenderer->frame_h);

    /* vycistit model matici */
    trLoadIdentity();

    /* nejprve nastavime posuv cele sceny od/ke kamere */
    trTranslate(0.0, 0.0, pRenderer->scene_move_z);

    /* pridame posuv cele sceny v rovine XY */
    trTranslate(pRenderer->scene_move_x, pRenderer->scene_move_y, 0.0);

    /* natoceni cele sceny - jen ve dvou smerech - mys je jen 2D... :( */
    trRotateX(pRenderer->scene_rot_x);
    trRotateY(pRenderer->scene_rot_y);

    /* nastavime material */
    renMatAmbient(pRenderer, &MAT_RED_AMBIENT);
    renMatDiffuse(pRenderer, &MAT_RED_DIFFUSE);
    renMatSpecular(pRenderer, &MAT_RED_SPECULAR);

    /* a vykreslime objekt ve stredu souradneho systemu */
    renderModel(pRenderer, pModel);

    /* ??? */
    /* ulozeni aktualni (referencni) matice */
    trGetMatrix(&savedMatrix);

    /* vnitrni kruznice */
    trScale(circle1Size, circle1Size, circle1Size); //nastaveni velikosti (prumeru)
    trTranslate(-1.0, pRenderer->scene_move_y, 0.0); //posun stredu do stredu souradneho systemu
    renderCurve(pRenderer, trajectory, COLOR_BLUE, CURVE_QUALITY); //vykresleni modrou barvou
    
    /* model na mensi kruznici */
    trSetMatrix(&savedMatrix); //nacteni referencni matice

    trRotateY(rotation1); //rotace kolem Y
    trTranslate(circle1Size, pRenderer->scene_move_y, 0.0); //posuv na kruznici
    trScale(model1Size, model1Size, model1Size); //nastaveni velikosti modelu

    renMatAmbient(pRenderer, &BLUE_AMBIENT); //nastaveni materialu (barvy)
    renMatDiffuse(pRenderer, &BLUE_DIFFUSE);
    renMatSpecular(pRenderer, &BLUE_SPECULAR);
    
    renderModel(pRenderer, pModel); //vykresleni modelu

    /* vnejsi kurznice */
    trSetMatrix(&savedMatrix); //nacteni referencni matice
    trScale(circle2Size, circle2Size, circle2Size); //nastaveni velikosti
    trTranslate(-1.0, pRenderer->scene_move_y, 0.0); //posun stredu do stredu souradneho systemu
    renderCurve(pRenderer, trajectory, COLOR_GREEN, CURVE_QUALITY); //vykresleni zelenou barvou

    /* model na vnejsi kruznici */ 
    trSetMatrix(&savedMatrix); //nacteni referencni matice

    trRotateY(rotation2); //rotace kolem osy Y
    trTranslate(circle2Size, pRenderer->scene_move_y, 0.0); //posun na kruznici
    trScale(model2Size, model2Size, model2Size);//nastaveni velikosti

    renMatAmbient(pRenderer, &GREEN_AMBIENT); //nastaveni materialu (barvy)
    renMatDiffuse(pRenderer, &GREEN_DIFFUSE);
    renMatSpecular(pRenderer, &GREEN_SPECULAR);

    renderModel(pRenderer, pModel); //vykresleni modelu

}


/*****************************************************************************
 *****************************************************************************/

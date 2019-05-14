#include "header.h"
#include "prototipo_func.h"
#include "structs.h"

int main()
{
    //definición de variables locales
    int i; //contador
    double H0[3]; //para guardar temporalmente los resultados instante cero
    double H1[3]; //para guardar temporalmente los resultados instante uno
    double H11[2];
    double H22[2];
    double op_flujo[2];
    //Despliegue de autoría en el terminal
    printf("****************************************************************************\n");
    printf("** Tarea 9, Belinda Brown B61254                                          **\n");
    printf("** Cálculo de dos posiciones bidimensionales y el vector de flujo óptico  **\n");
    printf("** Programa de referencia tomado del Prof. Dr.-Ing. Geovanni Martínez     **\n");
    printf("** IE-0449 Vision por Computador                                          **\n");
    printf("** I-2019                                                                 **\n");
    printf("****************************************************************************\n");
    printf("\n");

    //Reservando e inicializando memoria de contenedor p_parametros
    p_parametros = (struct contenedor_de_parametros_de_control *)malloc(sizeof(struct contenedor_de_parametros_de_control));
//puntero que direcciona a la posición de memoria para le instante cero
    p_parametros->GX0=0.0;
    p_parametros->GY0=0.0;
    p_parametros->GZ0=0.0;
    p_parametros->RX0=0.0;
    p_parametros->RY0=0.0;
    p_parametros->RZ0=0.0;
  //puntero que direcciona a la posición de memoria para le instante uno
    p_parametros->GX1=0.0;
    p_parametros->GY1=0.0;
    p_parametros->GZ1=0.0;
    p_parametros->RX1=0.0;
    p_parametros->RY1=0.0;
    p_parametros->RZ1=0.0;

    //puntero que direcciona a la posición de memoria para posicion del punto H respecto al sistema
    //de coordenadas de referencia
    p_parametros->HR=0.0;
    p_parametros->HS=0.0;
    p_parametros->HT=0.0;

    //puntero que direcciona a la posición de memoria para posicion de la distancia focal
    p_parametros->dis_focal=0.0;
    //Reservando e inicializando memoria de contenedor p_resultados
    p_resultados = (struct contenedor_de_resultados *)malloc(sizeof(struct contenedor_de_resultados));
    //Inicializando y reservando memoria para la posición tridemensional en el instante cero
    p_resultados->HX0=0.0;
    p_resultados->HY0=0.0;
    p_resultados->HZ0=0.0;
    //Inicializando y reservando memoria para la posición tridemensional en el instante uno
    p_resultados->HX1=0.0;
    p_resultados->HY1=0.0;
    p_resultados->HZ1=0.0;
    //Inicializando y reservando memoria para focal en el instante cero
    p_resultados->HX00=0.0;
    p_resultados->HY00=0.0;
    //Inicializando y reservando memoria para focal en el instante uno
    p_resultados->HX11=0.0;
    p_resultados->HY11=0.0;

    p_resultados->dx=0.0;
    p_resultados->dy=0.0;
    //Inicializando memoria de la matriz de rotacion R
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R0[i]=0.0;//instante uno
        p_resultados->R1[i]=0.0;//instante cero
    }
    //Reservando e inicializando memoria de la matriz de rotacion R2
//    p_resultados->R2=(double *)malloc(9*sizeof(double));
//    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
//        p_resultados->R2[i]=0.0;
  //  }

    //Esta función lee los parámetros de control de flujo del
    //programa desde un archivo de texto y los almacena en el
    //contenedor p_parametros
    geoLeerParametrosDeControlDeArchivoDeTexto();

    //Calculando matriz de rotacion instante cero
    geoObtenerMatrizDeRotacionR0();
    //Calculando matriz de rotacion instante uno
    geoObtenerMatrizDeRotacionR1();
    //Otra forma equivalente de calcular la matriz de rotacion
//    geoObtenerMatrizDeRotacionR2(p_parametros->RX, p_parametros->RY, p_parametros->RZ);

    // //Desplegando matriz de rotacion R
    // if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
    //     printf("\n"); //Linea en blanco
    //     printf("Matriz de rotacion R\n");
    //     printf("   r1=%f r2=%f r3=%f\n", p_resultados->R[0],p_resultados->R[1],p_resultados->R[2]);
    //     printf("   r4=%f r5=%f r6=%f\n", p_resultados->R[3],p_resultados->R[4],p_resultados->R[5]);
    //     printf("   r7=%f r8=%f r9=%f\n", p_resultados->R[6],p_resultados->R[7],p_resultados->R[8]);
    // }
    // else {
    //     printf("\n");
    //     printf("No se desplegara matriz de rotacion R\n");
    // }
    //
    // //Desplegando matriz de rotacion R2
    // if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
    //     printf("\n"); //Linea en blanco
    //     printf("Matriz de rotacion R2 (debe ser igual a R)\n");
    //     printf("   r1=%f r2=%f r3=%f\n", p_resultados->R2[0],p_resultados->R2[1],p_resultados->R2[2]);
    //     printf("   r4=%f r5=%f r6=%f\n", p_resultados->R2[3],p_resultados->R2[4],p_resultados->R2[5]);
    //     printf("   r7=%f r8=%f r9=%f\n", p_resultados->R2[6],p_resultados->R2[7],p_resultados->R2[8]);
    // }
    // else {
    //     printf("\n");
    //     printf("No se desplegara matriz de rotacion R2\n");
    // }

    //Calculando la posicion del punto respecto
    //a sistema de coordinadas de referencia
    //usando la transformacion de pose para el instante cero
    H0[0]=p_resultados->R0[0]*p_parametros->HR+p_resultados->R0[1]*p_parametros->HS+p_resultados->R0[2]*p_parametros->HT+p_parametros->GX0;
    H0[1]=p_resultados->R0[3]*p_parametros->HR+p_resultados->R0[4]*p_parametros->HS+p_resultados->R0[5]*p_parametros->HT+p_parametros->GY0;
    H0[2]=p_resultados->R0[6]*p_parametros->HR+p_resultados->R0[7]*p_parametros->HS+p_resultados->R0[8]*p_parametros->HT+p_parametros->GZ0;
   //Puntero hacia la localización de memoria de la posición tridimensional en el instante cero
    p_resultados->HX0=H0[0];
    p_resultados->HY0=H0[1];
    p_resultados->HZ0=H0[2];

    //usando la transformacion de pose para el instante uno
    H1[0]=p_resultados->R1[0]*p_parametros->HR+p_resultados->R1[1]*p_parametros->HS+p_resultados->R1[2]*p_parametros->HT+p_parametros->GX1;
    H1[1]=p_resultados->R1[3]*p_parametros->HR+p_resultados->R1[4]*p_parametros->HS+p_resultados->R1[5]*p_parametros->HT+p_parametros->GY1;
    H1[2]=p_resultados->R1[6]*p_parametros->HR+p_resultados->R1[7]*p_parametros->HS+p_resultados->R1[8]*p_parametros->HT+p_parametros->GZ1;
    //Puntero hacia la localización de memoria de la posición tridimensional en el instante uno
    p_resultados->HX1=H1[0];
    p_resultados->HY1=H1[1];
    p_resultados->HZ1=H1[2];

    //Se calcula proyecciones del punto en el plano de la cámara en el instante cero
    H11[0]=p_parametros->dis_focal*p_resultados->HX0/p_resultados->HZ0;
    H11[1]=p_parametros->dis_focal*p_resultados->HY0/p_resultados->HZ0;
    //Puntero hacia la localización de memoria de las proyecciones del punto en el plano de la cámara en el instante cero
    p_resultados->HX00=H11[0];
    p_resultados->HY00=H11[1];
    //Se calcula proyecciones del punto en el plano de la cámara en el instante uno
    H22[0]=p_parametros->dis_focal*p_resultados->HX1/p_resultados->HZ1;
    H22[1]=p_parametros->dis_focal*p_resultados->HY1/p_resultados->HZ1;
    //Puntero hacia la localización de memoria de las proyecciones del punto en el plano de la cámara en el instante uno   
    p_resultados->HX11=H22[0];
    p_resultados->HY11=H22[1];

    //Se calcula el vector de flujo óptico
    op_flujo[0]=p_resultados->HX11-p_resultados->HX00;
    op_flujo[1]=p_resultados->HY11-p_resultados->HY00;
    //Puntero hacia la localización de memoria del vector de flujo óptico
    p_resultados->dx=op_flujo[0];
    p_resultados->dy=op_flujo[1];

    //Desplegando en el terminal los resultados finales
    printf("\n"); //Linea en blanco
    printf("Resultado\n");
    printf("  Posicion bidimensional de la proyección del punto en el plano de la cámara en el instante cero\n");
    printf("   HX00=%f\n", p_resultados->HX00);
    printf("   HY00=%f\n", p_resultados->HY00);

    printf("  Posicion bidimensional de la proyección del punto en el plano de la cámara en el instante cero\n");
    printf("   HX11=%f\n", p_resultados->HX11);
    printf("   HY11=%f\n", p_resultados->HY11);


    printf("Vector de flujo optico entre los instantes de tiempo cero y uno\n");
    printf("(%f, %f)\n", p_resultados->dx, p_resultados->dy);

    //Salvando los resultados finales en archivo de texto
    geoSalvarResultadosEnArchivoDeTexto();

    //Liberando memoria reservada manualmente
    free(p_parametros);
    free(p_resultados);

    return 0;
}
//Fin de programa principal

//*******************************************************
//*******************************************************
//***** Introduzca aqui sus funciones               *****
//*******************************************************
//*******************************************************

void geoObtenerMatrizDeRotacionR0()
{
    double RX0, RY0, RZ0;

    //Conviertiendo angulos a radianes
    RX0=geoConvertirDeGradosARadianes(p_parametros->RX0);
    RY0=geoConvertirDeGradosARadianes(p_parametros->RY0);
    RZ0=geoConvertirDeGradosARadianes(p_parametros->RZ0);

    //Calculando la matriz de rotacion
    p_resultados->R0[0]=cos(RY0)*cos(RZ0);
    p_resultados->R0[1]=sin(RX0)*sin(RY0)*cos(RZ0)-cos(RX0)*sin(RZ0);
    p_resultados->R0[2]=cos(RX0)*sin(RY0)*cos(RZ0)+sin(RX0)*sin(RZ0);
    p_resultados->R0[3]=cos(RY0)*sin(RZ0);
    p_resultados->R0[4]=sin(RX0)*sin(RY0)*sin(RZ0)+cos(RX0)*cos(RZ0);
    p_resultados->R0[5]=cos(RX0)*sin(RY0)*sin(RZ0)-sin(RX0)*cos(RZ0);
    p_resultados->R0[6]=-sin(RY0);
    p_resultados->R0[7]=sin(RX0)*cos(RY0);
    p_resultados->R0[8]=cos(RX0)*cos(RY0);
}

void geoObtenerMatrizDeRotacionR1()
{
    double RX1, RY1, RZ1;
    //Conviertiendo angulos a radianes
    RX1=geoConvertirDeGradosARadianes(p_parametros->RX1);
    RY1=geoConvertirDeGradosARadianes(p_parametros->RY1);
    RZ1=geoConvertirDeGradosARadianes(p_parametros->RZ1);

    //Calculando la matriz de rotacion
    p_resultados->R1[0]=cos(RY1)*cos(RZ1);
    p_resultados->R1[1]=sin(RX1)*sin(RY1)*cos(RZ1)-cos(RX1)*sin(RZ1);
    p_resultados->R1[2]=cos(RX1)*sin(RY1)*cos(RZ1)+sin(RX1)*sin(RZ1);
    p_resultados->R1[3]=cos(RY1)*sin(RZ1);
    p_resultados->R1[4]=sin(RX1)*sin(RY1)*sin(RZ1)+cos(RX1)*cos(RZ1);
    p_resultados->R1[5]=cos(RX1)*sin(RY1)*sin(RZ1)-sin(RX1)*cos(RZ1);
    p_resultados->R1[6]=-sin(RY1);
    p_resultados->R1[7]=sin(RX1)*cos(RY1);
    p_resultados->R1[8]=cos(RX1)*cos(RY1);
}

double geoConvertirDeGradosARadianes(double angle)
{
    double res;

    res=angle*PI/180.0;

    return(res);
}

void geoLeerParametrosDeControlDeArchivoDeTexto()
{
    FILE *archivo;
    char d1[256], d2[256], d3[256];

    printf("Leyendo los datos de entrada:\n");

    //Abriendo archivo en mode de lectura
    char nombreDeArchivo[256]="current_control_parameters.txt";
    archivo = fopen(nombreDeArchivo, "r");
    if (!archivo) {
        printf("No se pudo abrir el archivo: current_control_parameters.txt\n");
        exit(1);
    }

    //Leyendo datos linea por linea

    //Brincando la primera y segunda lineas
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    fscanf(archivo, "\n");

    printf("  Posicion del objeto en el instante cero\n");

    //Leyendo GX0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GX0=(double)atof(d3);
    printf("   GX0: %f\n", p_parametros->GX0);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo GY0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GY0=(double)atof(d3);
    printf("   GY0: %f\n", p_parametros->GY0);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo GZ0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->GZ0=(double)atof(d3);
    printf("   GZ0: %f\n", p_parametros->GZ0);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    fscanf(archivo, "\n");
    printf("\n");
    printf("  Orientacion del objeto en el instante cero\n");

    //Leyendo RX0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RX0=(double)atof(d3);
    printf("   RX0: %f\n", p_parametros->RX0);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo RY0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RY0=(double)atof(d3);
    printf("   RY: %f\n", p_parametros->RY0);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo RZ0
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->RZ0=(double)atof(d3);
    printf("   RZ0: %f\n", p_parametros->RZ0);
    numeroDeDatosLeidos++;
    printf("\n");


        printf("  Posicion del objeto en el instante uno\n");

        //Leyendo GX1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->GX1=(double)atof(d3);
        printf("   GX1: %f\n", p_parametros->GX1);
        numeroDeDatosLeidos++;
        printf("\n");
        //Leyendo GY1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->GY1=(double)atof(d3);
        printf("   GY1: %f\n", p_parametros->GY1);
        numeroDeDatosLeidos++;
        printf("\n");
        //Leyendo GZ1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->GZ1=(double)atof(d3);
        printf("   GZ1: %f\n", p_parametros->GZ1);
        numeroDeDatosLeidos++;
        printf("\n");
        //Brincando linea de texto
        fscanf(archivo, "\n");

        printf("  Orientacion del objeto en el instante uno\n");

        //Leyendo RX1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->RX1=(double)atof(d3);
        printf("   RX1: %f\n", p_parametros->RX1);
        numeroDeDatosLeidos++;
        printf("\n");
        //Leyendo RY1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->RY1=(double)atof(d3);
        printf("   RY1: %f\n", p_parametros->RY1);
        numeroDeDatosLeidos++;
        printf("\n");

        //Leyendo RZ1
        fscanf(archivo, "%s %s %s\n", d1, d2, d3);
        p_parametros->RZ1=(double)atof(d3);
        printf("   RZ1: %f\n", p_parametros->RZ1);
        numeroDeDatosLeidos++;
        printf("\n");



    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Posicion del punto respecto a objeto\n");

    //Leyendo HR
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->HR=(double)atof(d3);
    printf("   HR: %f\n", p_parametros->HR);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo HS
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->HS=(double)atof(d3);
    printf("   HS: %f\n", p_parametros->HS);
    numeroDeDatosLeidos++;
    printf("\n");
    //Leyendo RT
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->HT=(double)atof(d3);
    printf("   HT: %f\n", p_parametros->HT);
    numeroDeDatosLeidos++;
    printf("\n");
    fscanf(archivo, "\n");

    printf("  Distancia focal\n");
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    p_parametros->dis_focal=(double)atof(d3);
    printf("   Resultado de distancia focal: %f\n", p_parametros->dis_focal);
    numeroDeDatosLeidos++;
    printf("\n");
    printf("  Numero de datos leidos: %d\n", numeroDeDatosLeidos);

    //Cerrando archivo
    fclose(archivo);
    //
    // printf("  Numero de datos leidos: %d\n", numeroDeDatosLeidos);
    //
    // //Cerrando archivo
    // fclose(archivo);
}

void geoSalvarResultadosEnArchivoDeTexto()
{
    FILE *archivo;
    int i;

    //Abriendo archivo en modo de escritura
    char nombreDeArchivo[256]="resultados.txt";
    archivo = fopen(nombreDeArchivo, "w");
    if (!archivo) {
        printf("No se pudo abrir el archivo: resultados.txt\n");
        exit(1);
    }

    //Escribiendo resultados en archivo
    //Guardando matriz de rotacion para el instante cero
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        fprintf(archivo, "Matriz de rotacion R0 para el instante cero\n");
        for (i=0;i<9;i++) {
            fprintf(archivo, "R0[%d]=%f\n", i, p_resultados->R0[i]);
        }
    }
    fprintf(archivo, "\n"); //linea en blanco
    //Guardando matriz de rotacion para el instante cero
    if (DESPLIEGUE_MATRIZ_DE_ROTACION==1) {
        fprintf(archivo, "Matriz de rotacion R1 para el instante uno\n");
        for (i=0;i<9;i++) {
            fprintf(archivo, "R1[%d]=%f\n", i, p_resultados->R1[i]);
        }
    }

    fprintf(archivo, "\n"); //linea en blanco


    //Guardando posicion
    fprintf(archivo, "Posicion bidimensional de la proyección del punto en el plano de la cámara en el instante cero\n");
    fprintf(archivo, "HX00=%f\n", p_resultados->HX00);
    fprintf(archivo, "HY00=%f\n", p_resultados->HY00);


    fprintf(archivo, "Posicion bidimensional de la proyección del punto en el plano de la cámara en el instante uno\n");
    fprintf(archivo, "HX11=%f\n", p_resultados->HX11);
    fprintf(archivo, "HY11=%f\n", p_resultados->HY11);


    fprintf(archivo, "Vector de flujo optico entre los instantes de tiempo cero y uno\n");
    fprintf(archivo, "(%f, %f)\n", p_resultados->dx, p_resultados->dy);


    // //Guardando posicion
    // fprintf(archivo, "Posicion del punto con respecto a referencia\n");
    // fprintf(archivo, "HX=%f\n", p_resultados->HX);
    // fprintf(archivo, "HY=%f\n", p_resultados->HY);
    // fprintf(archivo, "HZ=%f\n", p_resultados->HZ);

    //Cerrando archivo
    fclose(archivo);
}

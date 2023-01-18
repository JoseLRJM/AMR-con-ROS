#!/usr/bin/env python  

class templador:
    ID=[]
    num=0
    x=[]
    y=[]
    qz=[]
    qw=[]
    tagPosX=[] 
    tagAng=[] 
    tag_id=[]
    altura=[]
    tagPosY=[]

    for opm in range(1,100):
        ID.append(0)
        x.append(0.0)
        y.append(0.0)
        qz.append(0.0)
        qw.append(0.0)
        tagPosX.append(0.0) 
        tagAng.append(0.0) 
        tag_id.append(0)
        altura.append(0.0)
        tagPosY.append(0.0)

class maquina:  
    ID=[]
    num=0
    x=[]
    y=[]
    qz=[]
    qw=[]
    periodo=[]
    vel_botella=[]
    vel_stacker=[]
    templador_id=[]
    accion_id=[]       #0=maquina tag 1=,aquina_operador
    accion_boton_id=[]
    tagPosX=[]
    tagAng=[]
    tag_id=[]
    altura=[]
    tagPosY=[]
    tiempoAlerta=[]
    tiempoEspera=[]

    for pom in range(1,100):
        ID.append(0)
        x.append(0.0)
        y.append(0.0)
        qz.append(0.0)
        qw.append(0.0)
        periodo.append(0.0)
        vel_botella.append(0.0)
        vel_stacker.append(0.0)
        templador_id.append(0)
        accion_id.append(0)
        accion_boton_id.append(0)
        tagPosX.append(0.0)
        tagAng.append(0.0)
        tag_id.append(0.0)
        altura.append(0.0)
        tagPosY.append(0.0)
        tiempoAlerta.append(0.0)
        tiempoEspera.append(0.0)

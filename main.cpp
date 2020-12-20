//author  Renato Sousa, 2018
//Modificado por Zenio Angelo(zaon@cin.ufpe.br)
#include <bits/stdc++.h>
#include <QtNetwork>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "pb/messages_robocup_ssl_detection.pb.h"
#include "pb/messages_robocup_ssl_geometry.pb.h"
#include "pb/messages_robocup_ssl_wrapper.pb.h"
#include "pb/grSim_Packet.pb.h"
#include "pb/grSim_Commands.pb.h"
#include "pb/grSim_Replacement.pb.h"

class bola//classe que representa a bola
{
private:
    int id_bola;
    vector<pair<double, double>> coordenadas;//vector que guardará as posições da bola de acordo com cada câmera;indexado pelos IDs das câmeras
    queue<pair<double, double>> registro;//guardas as duas últimas localizações da bola

public:
    int contador_da_morte=0;//começa zerado; quando atinge 60, a posição da bola para de ser mostrada
    void ini(int id_bola);//função para inicializar o objeto
    void set_position(int id_camera, double x, double y);//guarda a posição da bola
    pair<double, double> get_position();//função que devolve, aproximadamente, a posição da bola no frame
};

void bola::ini(int id_bola){
        this->id_bola = id_bola;//atribui um id a bola
        coordenadas.push_back(make_pair(500000,500000));//coloca uma posição default para a câmera ID=0
        coordenadas.push_back(make_pair(500000,500000));
        coordenadas.push_back(make_pair(500000,500000));
        coordenadas.push_back(make_pair(500000,500000));
        registro.push(make_pair(0,0));//coloca duas posições iniciais, na fila registro, equivalentes ao centro do campo
        registro.push(make_pair(0,0));
    }
void bola::set_position(int id_camera, double x, double y){
    coordenadas[id_camera] = make_pair(x,y);//guarda a posição da bola de acordo com a câmera
}
pair<double, double> bola::get_position(){
    int count = 0;
    double soma_x = 0, soma_y = 0;
    for(int i = 0; i<4; i++){//soma as posições dadas pelas câmeras e conta quantas câmeras deram posições da bola
        if(coordenadas[i].first != 500000){//se a abscissa, da posição da bola, fornecida pela câmera "i" for diferente do valor deafult, então a cÂmera "i" forneceu uma posição para a bola
            soma_x += coordenadas[i].first;
            soma_y += coordenadas[i].second;
            coordenadas[i].first = 500000;
            coordenadas[i].second = 500000;
            count++; 
        }
    }
    if(count==0){//se nenhuma câmera deu a posição da bola, ela piscou, por isso, sua posição será suposta a partir de um vetor
        pair<double, double> POS_o = registro.front();//penúltima posição da bola
        registro.pop();
        pair<double, double> POS_f = registro.front();//última posição da bola
        registro.pop();
        registro.push(POS_f);//guarda a última posição na fila, pois esta posição será a penúltima e a antiga penúltima será descartada
        pair<double, double> vetor_bola = make_pair(POS_f.first - POS_o.first, POS_f.second - POS_o.second);//vetor = Posição_Final - Posição_Inicial
        registro.push(make_pair(POS_f.first + vetor_bola.first, POS_f.second + vetor_bola.second));//guardo a suposta nova posição da bola(POs_F + vetor_bola)
        contador_da_morte++;//como a bola piscou, o contador da morte dela é incrementado; se bater 60, presume-se que a bola não está mais em campo, pois ela não aparece há 60 frames.
        return make_pair(POS_f.first + vetor_bola.first, POS_f.second + vetor_bola.second);//retorno a suposta nova posição da bola(POs_F + vetor_bola)
    }
    else{//se uma ou mais câmeras deram a posição da bola, retiro a posição mais antiga e coloca a nova(média aritmética das posições entregues pela câmera)
        registro.pop();
        registro.push(make_pair(soma_x/count, soma_y/count));
        contador_da_morte = 0;
        return make_pair(soma_x/count, soma_y/count);//retorno a nova posição
    }
}

class robo//classe que representa os robôs; semelhante a classe bola, mas cada objeto tem seu contador da vida
{
private:
    vector<pair<double, double>> coordenadas;
    queue<pair<double, double>> registro;

public:
    int contador_da_morte=0, contador_da_vida=0, id_robo;
    double x=0, y=0;
    void ini(int id_robo);
    void set_position(int id_camera, double x, double y);
    pair<double, double> get_position();
};

void robo::ini(int id_robo){
        this->id_robo = id_robo;
        coordenadas.push_back(make_pair(500000,500000));
        coordenadas.push_back(make_pair(500000,500000));
        coordenadas.push_back(make_pair(500000,500000));
        coordenadas.push_back(make_pair(500000,500000));
        registro.push(make_pair(0,0));
        registro.push(make_pair(0,0));
    }
void robo::set_position(int id_camera, double x, double y){
    coordenadas[id_camera] = make_pair(x,y);
}
pair<double, double> robo::get_position(){
    int count = 0;
    double soma_x = 0, soma_y = 0;
    for(int i = 0; i<4; i++){
        if(coordenadas[i].first != 500000){
            soma_x += coordenadas[i].first;
            soma_y += coordenadas[i].second;
            coordenadas[i].first = 500000;
            coordenadas[i].second = 500000;
            count++; 
        }
    }
    if(count==0){
        pair<double, double> POS_o = registro.front();
        registro.pop();
        pair<double, double> POS_f = registro.front();
        registro.pop();
        registro.push(POS_f);
        pair<double, double> vetor_robo = make_pair(POS_f.first - POS_o.first, POS_f.second - POS_o.second);
        this->x = POS_f.first + vetor_robo.first;
        this->y = POS_f.second + vetor_robo.second;
        registro.push(make_pair(this->x, this->y));
        contador_da_morte++;
        contador_da_vida=0;
        return make_pair(this->x, this->y);
    }
    else{
        registro.pop();
        this->x = soma_x/count;
        this->y = soma_y/count;
        contador_da_morte = 0;
        registro.push(make_pair(this->x, this->y));
        return make_pair(this->x, this->y);
    }
}

void printRobotInfo(const SSL_DetectionRobot & robot) {
    printf("CONF=%4.2f ", robot.confidence());
    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(),robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }
    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());
}

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    bola *b = new bola();
    b->ini(0);//bola unica
    int contador_da_vida_BOLA = 0;//contador da vida da bola começa em zero, assim como com os robos
    pair<double, double> posicao;//pair que será usado para "printagem" de dados
    vector<robo*> robos_B, robos_Y;//vetor dinâmico para representar os times azul e amarelo
    for(int j = 0; j<10; j++){//inicializando os objetos em cada posicao; a categoria só usa seis robos, mas as cameras podem atribuir IDs maiores que 5, então botei 10 para evitar stack overflow
        robos_B.push_back(new robo());
        robos_B[j]->ini(j);
        robos_Y.push_back(new robo());
        robos_Y[j]->ini(j);
    }
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    GrSim_Client grSim_client;

    while(true) {
        if (client.receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();

                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %6.3fms\n",(detection.t_sent()-detection.t_capture())*600.0);
                printf("Network Latency (assuming synched system clock) %6.3fms\n",(t_now-detection.t_sent())*600.0);
                printf("Total Latency   (assuming synched system clock) %6.3fms\n",(t_now-detection.t_capture())*600.0);
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();

                //Ball info:
                for (int i = 0; i < balls_n; i++) {
                    if(b->contador_da_morte >= 60){//Se a bola não estiver em campo, mas uma suposta bola foi detectada, o contador de via é incrementado
                        contador_da_vida_BOLA++;
                    }
                    SSL_DetectionBall ball = detection.balls(i);
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> \n", i+1, balls_n, ball.confidence(),ball.x(),ball.y());                    
                    b->set_position(detection.camera_id(), ball.x(), ball.y());//guarda a posição da bola de acordo com a câmera
                    /*if (ball.has_z()) {
                        printf("Z=%6.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());*/
                }
                if(balls_n==0){//se a suposta bola piscar enquanto avalio se é realmente a bola, o contador de vida é zerado
                    contador_da_vida_BOLA = 0;
                }
                
                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot);
                    robos_B[robot.robot_id()]->set_position(detection.camera_id(), robot.x(), robot.y());//guardo a posição do robo no vector de acordo com a camera;uso do ID do robô para indexá-lo no vector
                    if(robos_B[robot.robot_id()]->contador_da_morte>=60){//se o robô não estiver em campo, mas um suposto robô de mesmo ID for detectado pela câmera, o contador da vida desde robô é incrementado
                        robos_B[robot.robot_id()]->contador_da_vida++;
                    }
                    /*if(robot.x() <= 0){
                        grSim_client.sendCommand(1.0, i);
                    }else{
                        grSim_client.sendCommand(-1.0, i);
                    }*/
                }

                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);
                    robos_Y[robot.robot_id()]->set_position(detection.camera_id(), robot.x(), robot.y());//mesma lógica dos robôs azuis
                    if(robos_Y[robot.robot_id()]->contador_da_morte>=60){
                        robos_Y[robot.robot_id()]->contador_da_vida++;
                    }
                }
                if(detection.camera_id() == 0){//Quando a quarta câmera mandar um frame
                    printf("\n");
                    if(b->contador_da_morte < 60){//A posição da bola só é apresentada se ela estiver presente em campo
                        posicao = b->get_position();
                        printf("--BOLA=<%.2lf,%.2lf>--\n", posicao.first, posicao.second);
                    }
                    else{//se a bola não estiver em campo, checa-se o contador da vida
                        if(contador_da_vida_BOLA >= 60){//se o contador da vida atingir 60, presume-se que a bola está, de fato, no campo e seu contador da morte é zerado, assim como seu contador da vida para futuras checagens
                            b->get_position();
                            b->contador_da_morte = 0;
                            contador_da_vida_BOLA = 0;
                        }
                    }
                    for(int i = 0; i<10; i++){//passando por todas as posições de robôs; são seis robôs, mas o ID não vem de 0 a 5.
                        if(robos_B[i]->contador_da_morte<60){//mesma lógica da bola
                            posicao = robos_B[i]->get_position();
                            printf("--RB%d=<%.2lf,%.2lf>--\n", i, posicao.first, posicao.second);
                        }
                        else{//mesma lógica da bola
                            if(robos_B[i]->contador_da_vida>=60){
                                robos_B[i]->get_position();
                                robos_B[i]->contador_da_morte = 0;
                                robos_B[i]->contador_da_vida = 0;
                            }
                        }
                    }
                    for(int i = 0; i<10; i++){//mesma lógica dos robôs azuis
                        if(robos_Y[i]->contador_da_morte<60){
                            posicao = robos_Y[i]->get_position();
                            printf("--RY%d=<%.2lf,%.2lf>--\n", i, posicao.first, posicao.second);
                        }
                        else{
                            if(robos_Y[i]->contador_da_vida>=60){
                                robos_Y[i]->get_position();
                                robos_Y[i]->contador_da_morte = 0;
                                robos_Y[i]->contador_da_vida = 0;
                            }
                        }
                    }
                    printf("\n");
                }
            }
            //see if packet contains geometry data:
            if (packet.has_geometry()) {
                const SSL_GeometryData & geom = packet.geometry();
                printf("-[Geometry Data]-------\n");

                const SSL_GeometryFieldSize & field = geom.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%d (mm)\n",field.field_length());
                printf("  -field_width=%d (mm)\n",field.field_width());
                printf("  -boundary_width=%d (mm)\n",field.boundary_width());
                printf("  -goal_width=%d (mm)\n",field.goal_width());
                printf("  -goal_depth=%d (mm)\n",field.goal_depth());
                printf("  -field_lines_size=%d\n",field.field_lines_size());
                printf("  -field_arcs_size=%d\n",field.field_arcs_size());

                int calib_n = geom.calib_size();
                for (int i=0; i< calib_n; i++) {
                    const SSL_GeometryCameraCalibration & calib = geom.calib(i);
                    printf("Camera Geometry for Camera ID %d:\n", calib.camera_id());
                    printf("  -focal_length=%.2f\n",calib.focal_length());
                    printf("  -principal_point_x=%.2f\n",calib.principal_point_x());
                    printf("  -principal_point_y=%.2f\n",calib.principal_point_y());
                    printf("  -distortion=%.2f\n",calib.distortion());
                    printf("  -q0=%.2f\n",calib.q0());
                    printf("  -q1=%.2f\n",calib.q1());
                    printf("  -q2=%.2f\n",calib.q2());
                    printf("  -q3=%.2f\n",calib.q3());
                    printf("  -tx=%.2f\n",calib.tx());
                    printf("  -ty=%.2f\n",calib.ty());
                    printf("  -tz=%.2f\n",calib.tz());

                    if (calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() && calib.has_derived_camera_world_tz()) {
                      printf("  -derived_camera_world_tx=%.f\n",calib.derived_camera_world_tx());
                      printf("  -derived_camera_world_ty=%.f\n",calib.derived_camera_world_ty());
                      printf("  -derived_camera_world_tz=%.f\n",calib.derived_camera_world_tz());
                    }

                }
            }
        }
    }
    return 0;
}
//Modificado por Zenio Angelo(zaon@cin.ufpe.br)
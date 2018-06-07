#include "../header/functions.h"

char mode_start;

void write_in_queue(RT_QUEUE *, MessageToMon);

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);


#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
      //  printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            //printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif

            err = send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
            if (err < 0)
                rt_sem_p(&sem_comLost, TM_INFINITE);
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        err = receive_message_from_monitor(msg.header, msg.data);
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }else if (msg.data[0] == CLOSE_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_closeComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);
            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
            }
        } else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            if (msg.data[0] == CAM_OPEN) {
                rt_sem_v(&sem_openCam);
            } else if (msg.data[0] == CAM_ASK_ARENA) {
                rt_sem_v(&sem_ask_arena);
            } else if (msg.data[0] == CAM_COMPUTE_POSITION) {
                computePos = true;
            } else if (msg.data[0] == CAM_STOP_COMPUTE_POSITION) {
                computePos = false;
            } else if (msg.data[0] == CAM_ARENA_CONFIRM) {         
                arenereponse = 1;        
                rt_sem_v(&sem_arena);
            } else if (msg.data[0] == CAM_ARENA_INFIRM) {
                arenereponse = 0;              
                rt_sem_v(&sem_arena);
            } else if (msg.data[0] == CAM_CLOSE) {
#ifdef _WITH_TRACE_
                printf("receive from monitor close cam\n");
#endif
                rt_sem_v(&sem_closeCam);
            }
        }
    } while (err > 0);
}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;
    MessageToMon msg;

    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
            failedCom = 0;
            rt_mutex_release(&mutex_failedCom);

            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
#ifdef _WITH_TRACE_
            printf("%s : the robot isnt started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
            failedCom++;
            rt_mutex_release(&mutex_failedCom);
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
        if (failedCom > 3) {
            close_communication_robot();
        }
    }
}

void f_move(void *arg) {
    int err;
    MessageToMon msg;
    
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            err = send_command_to_robot(move);
            rt_mutex_release(&mutex_move);
            
            if (err == 0) {
                rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
                failedCom = 0;
                rt_mutex_release(&mutex_failedCom);
                set_msgToMon_header(&msg, HEADER_STM_ACK);
                write_in_queue(&q_messageToMon, msg);
            } else {
                rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
                failedCom++;
                rt_mutex_release(&mutex_failedCom);
                set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                write_in_queue(&q_messageToMon, msg);
            }          
        }
        rt_mutex_release(&mutex_robotStarted);
        if (failedCom > 3) {
            close_communication_robot();
        }
    }
}

void f_checkBat(void * arg) {
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    
    /* PERIODIC START */

    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        MessageToMon msg;
        if (robotStarted) { 
            int reponse = send_command_to_robot(DMB_GET_VBAT);
            if (reponse < 0) {
                rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
                failedCom++;
                rt_mutex_release(&mutex_failedCom);
#ifdef _WITH_TRACE_
        printf("lost connection\n");
    #endif
                set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                write_in_queue(&q_messageToMon, msg);
            } else {
                rt_mutex_acquire(&mutex_failedCom, TM_INFINITE);
                failedCom = 0;
                rt_mutex_release(&mutex_failedCom);
               
                reponse = reponse + 48;
                set_msgToMon_header(&msg, HEADER_STM_BAT);
                set_msgToMon_data(&msg, &reponse);
                write_in_queue(&q_messageToMon, msg);
            } 
        }    
        rt_mutex_release(&mutex_robotStarted);
        if (failedCom > 3) {   
            close_communication_robot();
        }
    }
    
}

void f_openCam(void * arg) {
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);

    Image imageoriginal;
    Image imagesortie;
    Image imagePosition;
    Jpg imagecompress;
    MessageToMon msg;
    int err;
    int drawPos;

#ifdef _WITH_TRACE_
    printf("%s : Wait sem_openCam\n", info.name);
#endif
    rt_sem_p(&sem_openCam, TM_INFINITE);
#ifdef _WITH_TRACE_
    printf("%s : sem_openCam arrived => Start robot\n", info.name);
#endif

    if ((err = open_camera(&c)) == -1) {
        #ifdef _WITH_TRACE_
        printf("lost connection cam\n");
    #endif
        set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
        write_in_queue(&q_messageToMon, msg);
    } else {
        set_msgToMon_header(&msg, HEADER_STM_ACK);
        write_in_queue(&q_messageToMon, msg);
        rt_task_set_periodic(NULL, TM_NOW, 100000000);
        while (1) {
            rt_task_wait_period(NULL);
            //TODO variable partagé getimage
            if (getimage) {
                get_image(&c, &imageoriginal);
                
                if (computePos){
                    drawPos = detect_position(&imageoriginal,&p);
                    send_message_to_monitor(HEADER_STM_POS,&p);
                }
                
                if (drawArena) {
                    draw_arena(&imageoriginal, &imagesortie, &arene);
                    if (drawPos){
                        draw_position(&imagesortie,&imagePosition,&p);
                        send_message_to_monitor(HEADER_STM_POS,&p);
                        compress_image(&imagePosition, &imagecompress);
                    }
                    else{
                        compress_image(&imagesortie, &imagecompress);
                    }
                }
                else {
                    if (drawPos){
                        draw_position(&imageoriginal,&imagePosition,&p);
                        compress_image(&imagePosition, &imagecompress);
                    }
                    else{
                        compress_image(&imageoriginal, &imagecompress);
                    }
                }
                send_message_to_monitor(HEADER_STM_IMAGE, &imagecompress);
            }
        }
    }
}

void f_arene(void * arg) {
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    
    MessageToMon msg;
    Image imageoriginal, imagesortie;
    Jpg imagecompress;
    int err;
    
    while(1) {
        rt_sem_p(&sem_ask_arena,TM_INFINITE);

        // changer la variable partagée
        rt_mutex_acquire(&mutex_getimage, TM_INFINITE);
        getimage = false;
        rt_mutex_release(&mutex_getimage);

        get_image(&c, &imageoriginal);
        if (detect_arena(&imageoriginal, &arene) == -1) {
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            draw_arena(&imageoriginal, &imagesortie, &arene);
            Jpg imagecompress;
            compress_image(&imagesortie, &imagecompress);
            send_message_to_monitor(HEADER_STM_IMAGE, &imagecompress);
            rt_sem_p(&sem_arena, TM_INFINITE);
                 
            if (arenereponse == 0) {
                drawArena = false;   
            } else if (arenereponse == 1) {
                drawArena = true;
            }
        }
        
        rt_mutex_acquire(&mutex_getimage, TM_INFINITE);
        getimage = true;
        rt_mutex_release(&mutex_getimage);
    }
}

void f_closeCam(void * arg) {
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    
    rt_sem_p(&sem_closeCam, TM_INFINITE);
#ifdef _WITH_TRACE_
                printf("call close cam\n");
#endif
    close_camera(&c);
}

void f_closeComRobot(void * arg) {
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    
    rt_sem_p(&sem_closeComRobot, TM_INFINITE);
    close_communication_robot;
}

void f_close(void *arg) {	
	int err; 

	RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_comLost\n", info.name);
#endif
        rt_sem_p(&sem_comLost, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_comLost arrived => Closing\n", info.name);
#endif

        err = send_command_to_robot(DMB_STOP_MOVE);

        err = close_communication_robot();
        err = kill_nodejs();
        err = close_server();

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);
    }
}

void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
}
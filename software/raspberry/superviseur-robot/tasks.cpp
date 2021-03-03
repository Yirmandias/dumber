/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 35
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 18
#define PRIORITY_TSENDTOMON 25
#define PRIORITY_TRECEIVEFROMMON 30
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21


#define PRIORITY_TUPDATEBATTERY 5
#define PRIORITY_TOPENCAMERA 20
#define PRIORITY_TCLOSECAMERA 21
#define PRIORITY_TSENDIMAGE 17
#define PRIORITY_TFINDARENA 18
#define PRIORITY_TWD 19
#define PRIORITY_TCLOSEROBOT 21

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;
    
    //Attention, la caméra n'est pas encore initialisée
    d_maCamera = new CameraExtended(sm, 10);

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_robotConnected, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_maCamera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    if (err = rt_sem_create(&sem_robotStarted, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCapture, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_searchArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaAnswered, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    if (err = rt_sem_create(&sem_startRobotWithWatchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    if (err = rt_task_create(&th_updateBatteryLevel, "th_updateBatteryLevel", 0, PRIORITY_TUPDATEBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TOPENCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCLOSECAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImage, "th_sendImage", 0, PRIORITY_TSENDIMAGE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_findArena, "th_findArena", 0, PRIORITY_TFINDARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_SuperviseurWD, "th_SuperviseurWD", 0, PRIORITY_TWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeComRobot, "th_closeComRobot", 0, PRIORITY_TCLOSEROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*5000, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if ((err = rt_queue_create(&q_messageToArena, "q_messageToArena", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if ((err = rt_queue_create(&q_watchdog, "q_watchdog", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    if (err = rt_task_start(&th_updateBatteryLevel, (void(*)(void*)) & Tasks::UpdateBatteryLevel, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CloseCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImage, (void(*)(void*)) & Tasks::SendImage, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_findArena, (void(*)(void*)) & Tasks::FindArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_SuperviseurWD, (void(*)(void*)) & Tasks::SuperviseurWDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeComRobot, (void(*)(void*)) & Tasks::closeComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    Message *msgToSend = NULL;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            cout << "Communication between Supervisor and monitor LOST !" << endl << flush;
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD) || msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            WriteInQueue(&q_watchdog, new Message(msgRcv->GetID()));
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        //On nous demande d'ouvrir la caméra
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            //On envoie un sémaphore
            rt_sem_v(&sem_openCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            //On veut fermer la cam
            rt_sem_v(&sem_closeCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            //On cherche l'arene
            rt_sem_v(&sem_searchArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            //On refuse l'arene
            msgToSend = new Message(MESSAGE_CAM_ARENA_INFIRM);
            WriteInQueue(&q_messageToArena, msgToSend);
            rt_sem_v(&sem_arenaAnswered);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            //On accepte l'arene
            msgToSend = new Message(MESSAGE_CAM_ARENA_CONFIRM);
            WriteInQueue(&q_messageToArena, msgToSend);
            rt_sem_v(&sem_arenaAnswered);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            //On accepte l'arene
            rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
            d_maCamera->setFindPosition(true);
            rt_mutex_release(&mutex_maCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            //On accepte l'arene
            rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
            d_maCamera->setFindPosition(false);
            rt_mutex_release(&mutex_maCamera);
        }  
        
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    //Message to send to the robot
    Message * msgSend = new Message();
	Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        //rt_sem_p(&sem_startRobot, TM_INFINITE);
        
       msgRcv = ReadInQueue(&q_watchdog);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            cout << "Start robot with watchdog (";
            msgSend = robot.Write(robot.StartWithWD());
            rt_sem_v(&sem_startRobotWithWatchdog);

        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            cout << "Start robot without watchdog (";
            msgSend = robot.Write(robot.StartWithoutWD());
        }
        rt_mutex_release(&mutex_robot);
        
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            
            //On actualise la données robotConnected
            rt_mutex_acquire(&mutex_robotConnected, TM_INFINITE);
            d_robotConnected = true;
            rt_mutex_release(&mutex_robotConnected);
        }
        
        
        //On active le sémaphore
        //On l'envoie à tout le monde
        rt_sem_broadcast(&sem_robotStarted);
    }
}


/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    int cpComLost = 0;
    Message *msgRcv;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update"<< endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove<< endl << flush;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgRcv = robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
            if (msgRcv->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                cpComLost++;
                cout << "Com Lost, cpComLost= " << cpComLost << endl << flush;
                if (cpComLost == 3) {
                    cpComLost = 0;
                    cout << "Communication Lost" << endl << flush;
                    rt_sem_v(&sem_closeComRobot);

                }
            } else {
                cpComLost = 0;
            }
        }
    }
}





/**
 * 
 * Task to use the watchdog
 * @param arg
 */
void Tasks::SuperviseurWDTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    rt_sem_p(&sem_startRobotWithWatchdog, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update"<< endl << flush;
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        robot.Write(robot.ReloadWD());
        rt_mutex_release(&mutex_robot);
    }
}


/**
 * Task to close all the communication
 * @param arg
 */
void Tasks::closeComRobot(void* arg) {
    int status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_closeComRobot, TM_INFINITE);
        cout << "Close serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Close();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;
    }
}



/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}



//Thread pour afficher la batterie
void Tasks::UpdateBatteryLevel() {
    //On définit les variable sutilsées plus tard
    bool robotState = false;
    Message * levelBat = NULL;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1) {
        
        //On attend le sémaphore robotStarted
        rt_sem_p(&sem_robotStarted, TM_INFINITE);

        //On se met en mode périodique, pour toutes les 500ms
        rt_task_set_periodic(NULL, TM_NOW, 500000000); 

        while(1) {
            rt_task_wait_period(NULL);

            rt_mutex_acquire(&mutex_robotConnected, TM_INFINITE);
            robotState = d_robotConnected;
            rt_mutex_release(&mutex_robotConnected);

            if(robotState) {
                //On veut récupérer le niveau de batterie
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                levelBat = robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));     
                rt_mutex_release(&mutex_robot);

                WriteInQueue(&q_messageToMon, levelBat);

            } else {
                break;
            }
            
        }
     
    }

 }



//Thread pour ouvrir la caméra
void Tasks::OpenCamera() {
    
    bool allRight = false;
    Message * msgOpen = NULL;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1) {
    
        //On attend l'ordre d'ouverture de la caméra
        rt_sem_p(&sem_openCamera, TM_INFINITE);


        rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
        allRight = d_maCamera->Open();
        if(allRight) {
            //On actualise l'état de la cam
            d_maCamera->setCameraOpened(true);
            
            msgOpen = new Message(MESSAGE_ANSWER_ACK);
            WriteInQueue(&q_messageToMon, msgOpen);
            
            //On lance la capture d'img
            rt_sem_v(&sem_startCapture);
        } else {
            //Impossible de l'ouvrir
            d_maCamera->setCameraOpened(false);
            cout << "Opening the camera failed !" << endl << flush;
            
            msgOpen = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgOpen);
        }
        rt_mutex_release(&mutex_maCamera);
            
    } 
    
}




//Thread ton send images to the monitor
void Tasks::SendImage() {
    
    Img * myImage = NULL;
    
    std::list<Position> robotPosition;
    Position defaultPosition;
    defaultPosition.angle = 0;
    defaultPosition.robotId = -1;
    defaultPosition.center = cv::Point2f(-1, -1);
    defaultPosition.direction = cv::Point2f(0, 0);
    
    MessagePosition * msgPosition;
    MessageImg * msgImg;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    int i = 0;
    
    while(1) {
        
        //On attend le sémaphore startCapture
        rt_sem_p(&sem_startCapture, TM_INFINITE);
        
        //On se met en mode périodique, pour toutes les 100ms
        rt_task_set_periodic(NULL, TM_NOW, 100000000);
        
        while(1) {
            rt_task_wait_period(NULL);
            
            //On prend le mutex sur la caméra
            rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
            
            //On vérifie que la caméra soit connectée et que l'on ne recherche pas d'arène
            if(d_maCamera->getCameraOpened() && !(d_maCamera->getSearchingArena()) ) {
                //On récupère une image
                myImage = new Img(d_maCamera->Grab());
                //On initialise un nouveau message
                msgImg = new MessageImg();
                
                if(!d_maCamera->isArenaEmpty()) {
                    //On dessine l'arène si elle existe
                    myImage->DrawArena(d_maCamera->getSavedArena());
                }
                
                //Si on veut calculer la position et q'une arène est dispo
                if(d_maCamera->getFindPosition()) {
                    msgPosition = new MessagePosition();
                    
                    //On calcule la position
                    robotPosition = myImage->SearchRobot(d_maCamera->getSavedArena());
                    
                    if(robotPosition.empty()) {
                        msgPosition->SetPosition(defaultPosition);
                    } else {
                        msgPosition->SetPosition(robotPosition.front());
                        myImage->DrawAllRobots(robotPosition);
                    }
                    //On envoie le message avec la position
                    msgPosition->SetID(MESSAGE_CAM_POSITION);
                    WriteInQueue(&q_messageToMon, msgPosition); 
                }
                
                //On envoie l'image au moniteur
                msgImg->SetID(MESSAGE_CAM_IMAGE);
                msgImg->SetImage(myImage);
                WriteInQueue(&q_messageToMon, msgImg);
                
                rt_mutex_release(&mutex_maCamera);
            } else {
                rt_mutex_release(&mutex_maCamera);
                break;
            }
            
        }     
        
    }    
    
}



//Thread to close the camera
void Tasks::CloseCamera() {
    
    Arena * emptyArena = new Arena;
    Message * msgClose = NULL;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1) {
        
        //On attend le sémaphore closeCam
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        
        //On prend le mutex sur la caméra
        rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
        d_maCamera->Close();
        d_maCamera->setCameraOpened(false);
        d_maCamera->setFindPosition(false);
        d_maCamera->setSavedArena(*emptyArena);
        //On rend le mutex
        rt_mutex_release(&mutex_maCamera);
        
        //On envoie un msg de confirmation
        msgClose = new Message(MESSAGE_ANSWER_ACK);
        WriteInQueue(&q_messageToMon, msgClose);
       
        
    }
    
}



//Here we want to search the Arena
void Tasks::FindArena() {
    
    Arena foundArena;
    Message * msgFromMon = new Message();
    Message * msgToMon = new Message();
    MessageImg * msgImg;
    Img * myImage;
    
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    
    while(1) {
        //On attend qu'on demande l'attente d'arene
        rt_sem_p(&sem_searchArena, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_maCamera, TM_INFINITE);
        
            //On vérifie que la caméra soit connectée
            if(d_maCamera->getCameraOpened()) {
                
                //On se met en situation de recherche
                d_maCamera->setSearchingArena(true);
                
                myImage = new Img(d_maCamera->Grab());
                foundArena = myImage->SearchArena();
                
                //On dessine l'arène
                myImage->DrawArena(foundArena);
                //On envoie l'image au moniteur
                msgImg = new MessageImg();
                msgImg->SetID(MESSAGE_CAM_IMAGE);
                msgImg->SetImage(myImage);
                WriteInQueue(&q_messageToMon, msgImg); 
                    
                //On attend une réponse
                rt_sem_p(&sem_arenaAnswered, TM_INFINITE);
                msgFromMon = ReadInQueue(&q_messageToArena);
                    
                if(msgFromMon->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
                    //L'utilisateur accepte l'arène
                    //On enregistre la bonne arène
                    d_maCamera->setSavedArena(foundArena);
                    cout << "Arene acceptée" << endl << flush;
                } else if (msgFromMon->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
                    //L'utilisateur n'est pas OK
                    cout << "Arene refusee" << endl << flush;
                    //On supprime l'arène enregistrée précédemment
                    d_maCamera->setSavedArena(*(new Arena()));
                }
                             
                
                //La recherche est finie
                d_maCamera->setSearchingArena(false);
                
                rt_mutex_release(&mutex_maCamera);
                
            } else {
                //Dans le doute, on dit qu'il n'y a pas de recherche
                d_maCamera->setSearchingArena(false);
                rt_mutex_release(&mutex_maCamera);
                break;
            }
        
    }
    
}


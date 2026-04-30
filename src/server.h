#ifndef SERVER_H
#define SERVER_H

#include "datatypes.h"

/* 启动HTTP/WebSocket服务 (独立线程) */
void server_start(SimState *s, int port);

/* 停止服务 */
void server_stop(void);

/* 推送一帧数据到所有连接的客户端 */
void server_push_frame(SimState *s);

/* 构建JSON帧字符串 */
int  server_build_frame_json(SimState *s, char *buf, int buf_sz);

#endif

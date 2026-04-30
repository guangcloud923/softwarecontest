#include "server.h"

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef SOCKET socket_t;
  #define SOCK_INVALID INVALID_SOCKET
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  typedef int socket_t;
  #define SOCK_INVALID (-1)
  #define closesocket close
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CLIENTS 16
#define SERVER_BUF  65536

static SimState   *g_state = NULL;
static socket_t    g_listen = SOCK_INVALID;
static socket_t    g_clients[MAX_CLIENTS];
static int         g_client_cnt = 0;
static int         g_running = 0;
static int         g_port = 8080;

/* Base64编码 (用于WebSocket握手) */
static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode(const unsigned char *in, int len, char *out) {
    int i, j = 0;
    for (i = 0; i < len; i += 3) {
        unsigned int v = (unsigned int)in[i] << 16;
        if (i + 1 < len) v |= (unsigned int)in[i + 1] << 8;
        if (i + 2 < len) v |= (unsigned int)in[i + 2];
        out[j++] = B64[(v >> 18) & 0x3F];
        out[j++] = B64[(v >> 12) & 0x3F];
        out[j++] = (i + 1 < len) ? B64[(v >> 6) & 0x3F] : '=';
        out[j++] = (i + 2 < len) ? B64[v & 0x3F] : '=';
    }
    out[j] = '\0';
}

/* SHA1哈希 (简化, 仅用于WebSocket握手) */
typedef struct {
    unsigned int state[5];
    unsigned int count[2];
    unsigned char buf[64];
} SHA1_CTX;

#define SHA1_ROR(x,n) (((x) >> (n)) | ((x) << (32 - (n))))

static void sha1_transform(unsigned int state[5], const unsigned char buf[64]) {
    unsigned int w[80], a, b, c, d, e, t;
    int i;
    for (i = 0; i < 16; i++)
        w[i] = ((unsigned int)buf[i*4] << 24) | ((unsigned int)buf[i*4+1] << 16) |
               ((unsigned int)buf[i*4+2] << 8) | (unsigned int)buf[i*4+3];
    for (i = 16; i < 80; i++)
        w[i] = SHA1_ROR(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);
    a = state[0]; b = state[1]; c = state[2]; d = state[3]; e = state[4];
    for (i = 0; i < 80; i++) {
        if (i < 20) t = (b & c) | (~b & d);
        else if (i < 40) t = b ^ c ^ d;
        else if (i < 60) t = (b & c) | (b & d) | (c & d);
        else t = b ^ c ^ d;
        t += SHA1_ROR(a, 5) + e + w[i];
        if (i < 20) t += 0x5A827999;
        else if (i < 40) t += 0x6ED9EBA1;
        else if (i < 60) t += 0x8F1BBCDC;
        else t += 0xCA62C1D6;
        e = d; d = c; c = SHA1_ROR(b, 30); b = a; a = t;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d; state[4] += e;
}

static void sha1_init(SHA1_CTX *ctx) {
    ctx->state[0] = 0x67452301; ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE; ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count[0] = ctx->count[1] = 0;
}

static void sha1_update(SHA1_CTX *ctx, const unsigned char *data, int len) {
    int i, j = ctx->count[0];
    ctx->count[0] += len * 8;
    if (ctx->count[0] < (unsigned int)(len * 8)) ctx->count[1]++;
    ctx->count[1] += (len * 8) >> 32;
    for (i = 0; i < len; i++) {
        ctx->buf[j++] = data[i];
        if (j == 64) { sha1_transform(ctx->state, ctx->buf); j = 0; }
    }
}

static void sha1_final(SHA1_CTX *ctx, unsigned char digest[20]) {
    int i, j = ctx->count[0] / 8 % 64;
    ctx->buf[j++] = 0x80;
    if (j > 56) {
        for (; j < 64; j++) ctx->buf[j] = 0;
        sha1_transform(ctx->state, ctx->buf);
        j = 0;
    }
    for (; j < 56; j++) ctx->buf[j] = 0;
    for (i = 0; i < 8; i++)
        ctx->buf[56 + i] = (ctx->count[1] >> (56 - i * 8)) & 0xFF;
    sha1_transform(ctx->state, ctx->buf);
    for (i = 0; i < 20; i++)
        digest[i] = (ctx->state[i >> 2] >> (24 - (i & 3) * 8)) & 0xFF;
}

/* WebSocket握手 */
static int ws_handshake(socket_t client, const char *request) {
    const char *key_start = strstr(request, "Sec-WebSocket-Key: ");
    if (!key_start) return 0;
    key_start += 19;
    const char *key_end = strstr(key_start, "\r\n");
    if (!key_end) return 0;

    char key[256];
    int klen = (int)(key_end - key_start);
    if (klen >= 255) klen = 255;
    memcpy(key, key_start, klen);
    key[klen] = '\0';

    /* 拼接魔数 */
    char concat[512];
    snprintf(concat, sizeof(concat), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);

    /* SHA1 */
    SHA1_CTX ctx;
    unsigned char hash[20];
    sha1_init(&ctx);
    sha1_update(&ctx, (unsigned char *)concat, (int)strlen(concat));
    sha1_final(&ctx, hash);

    /* Base64 */
    char accept[64];
    base64_encode(hash, 20, accept);

    /* 发送101响应 */
    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "\r\n", accept);

    send(client, response, (int)strlen(response), 0);
    return 1;
}

/* 发送WebSocket文本帧 */
static void ws_send(socket_t client, const char *data, int len) {
    unsigned char frame[10];
    int hdr_sz;

    frame[0] = 0x81; /* FIN + text */
    if (len < 126) {
        frame[1] = (unsigned char)len;
        hdr_sz = 2;
    } else if (len < 65536) {
        frame[1] = 126;
        frame[2] = (len >> 8) & 0xFF;
        frame[3] = len & 0xFF;
        hdr_sz = 4;
    } else {
        frame[1] = 127;
        memset(&frame[2], 0, 6);
        frame[6] = (len >> 24) & 0xFF;
        frame[7] = (len >> 16) & 0xFF;
        frame[8] = (len >> 8) & 0xFF;
        frame[9] = len & 0xFF;
        hdr_sz = 10;
    }

    send(client, (const char *)frame, hdr_sz, 0);
    send(client, data, len, 0);
}

/* 构建JSON帧 */
int server_build_frame_json(SimState *s, char *buf, int buf_sz) {
    int pos = 0;
    pos += snprintf(buf + pos, buf_sz - pos,
        "{\"frame_id\":%d,\"time\":%.1f,\"t_score\":%.1f,\"running\":%s,\"difficulty\":%.1f,",
        s->frame_id, s->time, s->score, s->running ? "true" : "false", s->difficulty);

    /* AGVs */
    pos += snprintf(buf + pos, buf_sz - pos, "\"agvs\":[");
    int i, first = 1;
    for (i = 0; i < s->agv_cnt; i++) {
        AGV *a = &s->agvs[i];
        const char *st = "IDLE";
        switch (a->status) {
            case AGV_MOVING_TO_BUFFER: st = "MOVING"; break;
            case AGV_MOVING_TO_TRANSFER: st = "MOVING"; break;
            case AGV_LOADING: st = "LOADING"; break;
            case AGV_UNLOADING: st = "UNLOADING"; break;
            case AGV_WAITING: st = "WAITING"; break;
            default: break;
        }
        if (!first) pos += snprintf(buf + pos, buf_sz - pos, ",");
        first = 0;
        pos += snprintf(buf + pos, buf_sz - pos,
            "{\"id\":\"AGV_%02d\",\"pos\":[%d,%d],\"status\":\"%s\",\"load\":%.2f,\"items\":[",
            a->id, a->pos.x, a->pos.y, st, a->load_vol);
        int j, f2 = 1;
        for (j = 0; j < a->item_cnt; j++) {
            if (!f2) pos += snprintf(buf + pos, buf_sz - pos, ",");
            f2 = 0;
            Item *it = &s->items[a->items[j]];
            pos += snprintf(buf + pos, buf_sz - pos,
                "{\"id\":\"ITEM_%d\",\"vol\":%.2f}", it->id, VOL_VAL[it->volume]);
        }
        pos += snprintf(buf + pos, buf_sz - pos, "]}");
    }
    pos += snprintf(buf + pos, buf_sz - pos, "],");

    /* Buffers */
    pos += snprintf(buf + pos, buf_sz - pos, "\"buffers\":{");
    for (i = 0; i < s->buffer_cnt; i++) {
        if (i > 0) pos += snprintf(buf + pos, buf_sz - pos, ",");
        pos += snprintf(buf + pos, buf_sz - pos, "\"buffer_%d\":[", i);
        int j;
        for (j = 0; j < s->buffers[i].item_cnt; j++) {
            if (j > 0) pos += snprintf(buf + pos, buf_sz - pos, ",");
            pos += snprintf(buf + pos, buf_sz - pos,
                "\"ITEM_%d\"", s->buffers[i].items[j]);
        }
        pos += snprintf(buf + pos, buf_sz - pos, "]");
    }
    pos += snprintf(buf + pos, buf_sz - pos, "},");

    /* Transfer zones */
    pos += snprintf(buf + pos, buf_sz - pos, "\"transfers\":[");
    for (i = 0; i < s->tzone_cnt; i++) {
        if (i > 0) pos += snprintf(buf + pos, buf_sz - pos, ",");
        pos += snprintf(buf + pos, buf_sz - pos,
            "{\"id\":%d,\"vol\":%.2f,\"items\":%d}", i, s->tzones[i].total_vol, s->tzones[i].item_cnt);
    }
    pos += snprintf(buf + pos, buf_sz - pos, "],");

    /* Events (最近5条) */
    pos += snprintf(buf + pos, buf_sz - pos, "\"events\":[");
    int ev_start = s->event_cnt > 5 ? s->event_cnt - 5 : 0;
    for (i = ev_start; i < s->event_cnt; i++) {
        if (i > ev_start) pos += snprintf(buf + pos, buf_sz - pos, ",");
        pos += snprintf(buf + pos, buf_sz - pos, "\"%s\"", s->events[i].msg);
    }
    pos += snprintf(buf + pos, buf_sz - pos, "],");

    /* Stats */
    pos += snprintf(buf + pos, buf_sz - pos,
        "\"stats\":{\"shelved\":%d,\"total\":%d,\"violations\":%d,\"collision\":%s},",
        s->items_shelved, s->items_total, s->violations,
        s->collision_flag ? "true" : "false");

    /* Grid map (static, sent each frame for client convenience) */
    pos += snprintf(buf + pos, buf_sz - pos, "\"grid\":[");
    int y, x, fr = 1;
    for (y = 0; y < GRID_H; y++) {
        if (!fr) pos += snprintf(buf + pos, buf_sz - pos, ",");
        fr = 0;
        pos += snprintf(buf + pos, buf_sz - pos, "[");
        for (x = 0; x < GRID_W; x++) {
            if (x > 0) pos += snprintf(buf + pos, buf_sz - pos, ",");
            pos += snprintf(buf + pos, buf_sz - pos, "%d", s->grid[y][x].type);
        }
        pos += snprintf(buf + pos, buf_sz - pos, "]");
    }
    pos += snprintf(buf + pos, buf_sz - pos, "]");

    pos += snprintf(buf + pos, buf_sz - pos, "}\n");
    return pos;
}

/* 处理HTTP请求 */
static void handle_http(socket_t client, const char *request) {
    char response[SERVER_BUF];
    if (strstr(request, "GET /api/state")) {
        char json[SERVER_BUF / 2];
        int jlen = server_build_frame_json(g_state, json, sizeof(json));
        snprintf(response, sizeof(response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s", jlen, json);
        send(client, response, (int)strlen(response), 0);
    } else if (strstr(request, "GET / ") || strstr(request, "GET /visualization")) {
        /* Serve the visualization HTML page */
        const char *html =
            "<!DOCTYPE html>\n<html lang=\"zh-CN\">\n<head>\n"
            "<meta charset=\"UTF-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
            "<title>Warehouse Sim</title>\n"
            "<style>\n"
            "*{margin:0;padding:0;box-sizing:border-box}\n"
            "body{background:#1a1a2e;color:#e0e0e0;font-family:Consolas,monospace;display:flex;flex-direction:column;height:100vh}\n"
            "#bar{display:flex;justify-content:space-between;align-items:center;padding:6px 16px;background:#0f3460;font-size:12px}\n"
            "#bar .dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:6px}\n"
            ".g{background:#0f0}.r{background:#f00}\n"
            "#bar input{background:#1a1a2e;border:1px solid #333;color:#e0e0e0;padding:3px 6px;border-radius:3px;width:200px;font:12px Consolas,monospace}\n"
            "#bar button{background:#e94560;color:#fff;border:none;padding:3px 10px;border-radius:3px;cursor:pointer;font:12px Consolas,monospace}\n"
            "#main{display:flex;flex:1;padding:12px;gap:12px;overflow:hidden}\n"
            "#grid-wrap{flex:1;display:flex;flex-direction:column;align-items:center}\n"
            "#grid-wrap h2{color:#00d4ff;margin-bottom:6px;font-size:16px;letter-spacing:2px}\n"
            "#side{width:300px;display:flex;flex-direction:column;gap:8px;overflow-y:auto}\n"
            ".box{background:#16213e;border:1px solid #0f3460;border-radius:6px;padding:10px}\n"
            ".box h3{color:#e94560;font-size:12px;margin-bottom:6px;text-transform:uppercase;letter-spacing:1px}\n"
            ".row{display:flex;justify-content:space-between;padding:2px 0;font-size:12px}\n"
            ".row .l{color:#888}.row .v{color:#00d4ff;font-weight:bold}\n"
            ".bar{background:#0f3460;height:5px;border-radius:3px;margin:4px 0;overflow:hidden}\n"
            ".bar div{height:100%;background:linear-gradient(90deg,#00d4ff,#e94560);border-radius:3px;transition:width .3s}\n"
            "#events{max-height:130px;overflow-y:auto;font-size:10px}\n"
            "#events .ev{padding:1px 0;border-bottom:1px solid #1a1a2e;color:#aaa}\n"
            "#leg{display:flex;flex-wrap:wrap;gap:6px;font-size:10px;margin-top:8px}\n"
            "#leg span{display:flex;align-items:center;gap:3px}\n"
            "#leg .sw{width:10px;height:10px;border-radius:2px}\n"
            "</style>\n</head>\n<body>\n"
            "<div id=\"bar\">\n"
            "<div><span id=\"dot\" class=\"r dot\"></span><span id=\"stat\">offline</span>"
            " &nbsp; F:<b id=\"fid\">-</b> &nbsp; T:<b id=\"st\">0</b></div>\n"
            "<div><input id=\"url\" value=\"ws://localhost:8080/ws\"><button id=\"bn\" onclick=\"cn()\">connect</button></div>\n"
            "</div>\n"
            "<div id=\"main\">\n"
            "<div id=\"grid-wrap\"><h2>WAREHOUSE SIM</h2><canvas id=\"cv\"></canvas>\n"
            "<div id=\"leg\">"
            "<span><div class=\"sw\" style=\"background:#333\"></div>road</span>"
            "<span><div class=\"sw\" style=\"background:#ff9800\"></div>conveyor</span>"
            "<span><div class=\"sw\" style=\"background:#4caf50\"></div>buffer</span>"
            "<span><div class=\"sw\" style=\"background:#9c27b0\"></div>junction</span>"
            "<span><div class=\"sw\" style=\"background:#2196f3\"></div>transfer</span>"
            "<span><div class=\"sw\" style=\"background:#ff5722\"></div>shelf</span>"
            "<span><div class=\"sw\" style=\"background:#607d8b\"></div>park</span>"
            "<span><div class=\"sw\" style=\"background:#00d4ff;border-radius:50%\"></div>AGV</span>"
            "</div></div>\n"
            "<div id=\"side\">"
            "<div class=\"box\"><h3>Stats</h3>"
            "<div class=\"row\"><span class=\"l\">Difficulty</span><span class=\"v\" id=\"sd\">-</span></div>"
            "<div class=\"row\"><span class=\"l\">Shelved/Total</span><span class=\"v\" id=\"ss\">-</span></div>"
            "<div class=\"bar\"><div id=\"sb\" style=\"width:0%\"></div></div>"
            "<div class=\"row\"><span class=\"l\">Score</span><span class=\"v\" id=\"sc\">-</span></div>"
            "<div class=\"row\"><span class=\"l\">Violations</span><span class=\"v\" id=\"sv\">0</span></div>"
            "<div class=\"row\"><span class=\"l\">Collision</span><span class=\"v\" id=\"sx\">-</span></div></div>"
            "<div class=\"box\"><h3>AGVs</h3><div id=\"agvs\"></div></div>"
            "<div class=\"box\"><h3>Buffers / Transfers</h3><div id=\"bt\"></div></div>"
            "<div class=\"box\"><h3>Events</h3><div id=\"events\"></div></div>"
            "</div></div>\n"
            "<script>\n"
            "const CS=42,W=16,H=14;\n"
            "const CC={0:'#222',1:'#111',2:'#ff9800',3:'#4caf50',4:'#9c27b0',5:'#2196f3',6:'#ff5722',7:'#607d8b',8:'#555'};\n"
            "const CL={2:'C',3:'B',4:'J',5:'T',6:'S',7:'P'};\n"
            "const AC=['#00d4ff','#ffeb3b','#ff6b81','#69f0ae'];\n"
            "let ws,grd=null,fd=null,cv,ctx;\n"
            "const VR=[1,3,5,7,10,12,14],HR=[0,2,4,6,8,10];\n"
            "const BP=[[3,2],[12,2]],TP=[[1,8],[5,8],[10,8],[14,8]];\n"
            "function init(){\n"
            "cv=document.getElementById('cv');cv.width=W*CS;cv.height=H*CS;ctx=cv.getContext('2d');\n"
            "drg();}\n"
            "function drg(){\n"
            "ctx.fillStyle='#111';ctx.fillRect(0,0,cv.width,cv.height);\n"
            "VR.forEach(x=>{HR.forEach(y=>{ctx.fillStyle='#2a2a3a';ctx.fillRect(x*CS,y*CS,CS,CS);});\n"
            "for(let y=0;y<H;y++){if(!HR.includes(y)){ctx.fillStyle='#2a2a3a';ctx.fillRect(x*CS,y*CS,CS,CS);}}});\n"
            "ctx.strokeStyle='#1a1a2e';ctx.lineWidth=1;\n"
            "for(let x=0;x<=W;x++){ctx.beginPath();ctx.moveTo(x*CS,0);ctx.lineTo(x*CS,H*CS);ctx.stroke();}\n"
            "for(let y=0;y<=H;y++){ctx.beginPath();ctx.moveTo(0,y*CS);ctx.lineTo(W*CS,y*CS);ctx.stroke();}\n"
            "}\n"
            "function draw(){\n"
            "if(!grd)return;drg();\n"
            "for(let y=0;y<H;y++)for(let x=0;x<W;x++){\n"
            "let t=grd[y][x];if(t===0||t===1)continue;\n"
            "ctx.fillStyle=CC[t]||'#333';ctx.fillRect(x*CS+2,y*CS+2,CS-4,CS-4);\n"
            "if(CL[t]){ctx.fillStyle='#fff';ctx.font='bold 14px Consolas';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(CL[t],x*CS+CS/2,y*CS+CS/2);}}\n"
            "if(!fd)return;\n"
            "if(fd.agvs)fd.agvs.forEach((a,i)=>{\n"
            "let cx=a.pos[0]*CS+CS/2,cy=a.pos[1]*CS+CS/2,r=CS/2-4;\n"
            "ctx.beginPath();ctx.arc(cx,cy,r+2,0,6.28);ctx.fillStyle=AC[i]+'44';ctx.fill();\n"
            "ctx.beginPath();ctx.arc(cx,cy,r,0,6.28);ctx.fillStyle=AC[i];ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();\n"
            "ctx.fillStyle='#000';ctx.font='bold 10px Consolas';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('A'+i,cx,cy);\n"
            "if(a.load>0){ctx.fillStyle='#fff';ctx.font='8px Consolas';ctx.fillText(a.load.toFixed(1),cx,cy+r+10);}});\n"
            "if(fd.buffers)BP.forEach(([bx,by],i)=>{\n"
            "let k='buffer_'+i,cnt=fd.buffers[k]?fd.buffers[k].length:0;\n"
            "if(cnt>0){ctx.fillStyle='#fff';ctx.font='bold 11px Consolas';ctx.textAlign='center';ctx.fillText(cnt+')',bx*CS+CS/2,by*CS+CS-8);}});\n"
            "if(fd.transfers)fd.transfers.forEach((tz,i)=>{\n"
            "if(i<4){ctx.fillStyle='#fff';ctx.font='bold 10px Consolas';ctx.textAlign='center';ctx.fillText(tz.vol.toFixed(1)+'m',TP[i][0]*CS+CS/2,TP[i][1]*CS+CS-8);}});\n"
            "}\n"
            "function upd(){\n"
            "if(!fd)return;let d=fd;\n"
            "document.getElementById('fid').textContent=d.frame_id;\n"
            "document.getElementById('st').textContent=d.time;\n"
            "document.getElementById('sd').textContent=(d.difficulty||0).toFixed(1);\n"
            "document.getElementById('sc').textContent=(d.t_score||0).toFixed(1);\n"
            "let s=d.stats||{};\n"
            "document.getElementById('ss').textContent=(s.shelved||0)+'/'+(s.total||0);\n"
            "let rt=s.total>0?(s.shelved/s.total*100).toFixed(1):0;\n"
            "document.getElementById('sb').style.width=rt+'%';\n"
            "document.getElementById('sv').textContent=s.violations||0;\n"
            "document.getElementById('sx').textContent=s.collision?'COLLISION':'OK';\n"
            "let ah='';if(d.agvs)d.agvs.forEach((a,i)=>{ah+=`<div class=\"row\" style=\"color:${AC[i]}\"><span>AGV${i} (${a.pos[0]},${a.pos[1]})</span><span>${a.status}|${a.load.toFixed(2)}m|${(a.items||[]).length}it</span></div>`;});\n"
            "document.getElementById('agvs').innerHTML=ah||'<div class=\"row\"><span class=\"l\">-</span></div>';\n"
            "let bh='';if(d.buffers)for(let i=0;i<2;i++){let k='buffer_'+i,cnt=d.buffers[k]?d.buffers[k].length:0;bh+=`<div class=\"row\"><span class=\"l\">B${i+1}</span><span class=\"v\">${cnt}/6</span></div>`;}\n"
            "if(d.transfers)d.transfers.forEach((tz,i)=>{bh+=`<div class=\"row\"><span class=\"l\">T${i+1}</span><span class=\"v\">${tz.items||0}it|${tz.vol.toFixed(2)}m</span></div>`;});\n"
            "document.getElementById('bt').innerHTML=bh||'<div class=\"row\"><span class=\"l\">-</span></div>';\n"
            "if(d.events&&d.events.length){let eh='';d.events.forEach(e=>{eh+=`<div class=\"ev\">${e}</div>`;});document.getElementById('events').innerHTML=eh;}\n"
            "if(d.running===false)document.getElementById('stat').textContent='finished';\n"
            "}\n"
            "function onmsg(e){let d=JSON.parse(e.data);if(d.grid)grd=d.grid;fd=d;draw();upd();}\n"
            "function cn(){\n"
            "let url=document.getElementById('url').value;if(ws)ws.close();\n"
            "ws=new WebSocket(url);\n"
            "ws.onopen=()=>{document.getElementById('dot').className='g dot';document.getElementById('stat').textContent='connected';document.getElementById('bn').textContent='disconnect';document.getElementById('bn').onclick=()=>ws.close();};\n"
            "ws.onclose=()=>{document.getElementById('dot').className='r dot';document.getElementById('stat').textContent='offline';document.getElementById('bn').textContent='connect';document.getElementById('bn').onclick=cn;};\n"
            "ws.onmessage=onmsg;\n"
            "}\n"
            "init();cn();\n"
            "</script>\n</body>\n</html>\n";
        int hlen = (int)strlen(html);
        snprintf(response, sizeof(response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s", hlen, html);
        send(client, response, (int)strlen(response), 0);
    } else {
        const char *body = "{\"status\":\"warehouse_sim running\",\"ws\":\"ws://localhost:8080/ws\"}";
        snprintf(response, sizeof(response),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\nContent-Length: %d\r\n\r\n%s",
            (int)strlen(body), body);
        send(client, response, (int)strlen(response), 0);
    }
}

/* 处理新客户端 */
static void handle_client(socket_t client) {
    char buf[4096];
    int n = recv(client, buf, sizeof(buf) - 1, 0);
    if (n <= 0) { closesocket(client); return; }
    buf[n] = '\0';

    if (strstr(buf, "Upgrade: websocket")) {
        if (ws_handshake(client, buf)) {
            /* 添加客户端 */
            int i;
            for (i = 0; i < MAX_CLIENTS; i++) {
                if (g_clients[i] == SOCK_INVALID) {
                    g_clients[i] = client;
                    g_client_cnt++;
                    return;
                }
            }
        }
    } else if (strstr(buf, "GET ")) {
        handle_http(client, buf);
    }
    closesocket(client);
}

void server_push_frame(SimState *s) {
    if (!g_running) return;

    s->frame_id++;

    char json[SERVER_BUF / 2];
    int jlen = server_build_frame_json(s, json, sizeof(json));
    if (jlen <= 0) return;

    int i;
    for (i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i] != SOCK_INVALID) {
            ws_send(g_clients[i], json, jlen);
        }
    }
}

static void server_loop(void) {
    fd_set readfds;
    struct timeval tv;

    while (g_running) {
        FD_ZERO(&readfds);
        FD_SET(g_listen, &readfds);
        int max_fd = (int)g_listen;
        int i;

        for (i = 0; i < MAX_CLIENTS; i++) {
            if (g_clients[i] != SOCK_INVALID) {
                FD_SET(g_clients[i], &readfds);
                if ((int)g_clients[i] > max_fd) max_fd = (int)g_clients[i];
            }
        }

        tv.tv_sec = 0;
        tv.tv_usec = 16000; /* ~60fps */
        int ret = select(max_fd + 1, &readfds, NULL, NULL, &tv);

        if (ret < 0) break;
        if (ret == 0) continue;

        if (FD_ISSET(g_listen, &readfds)) {
            struct sockaddr_in addr;
            socklen_t addr_len = sizeof(addr);
            socket_t client = accept(g_listen, (struct sockaddr *)&addr, &addr_len);
            if (client != SOCK_INVALID) {
#ifdef _WIN32
                unsigned long mode = 1;
                ioctlsocket(client, FIONBIO, &mode);
#else
                int flags = fcntl(client, F_GETFL, 0);
                fcntl(client, F_SETFL, flags | O_NONBLOCK);
#endif
                handle_client(client);
            }
        }

        for (i = 0; i < MAX_CLIENTS; i++) {
            if (g_clients[i] != SOCK_INVALID && FD_ISSET(g_clients[i], &readfds)) {
                char buf[256];
                int n = recv(g_clients[i], buf, sizeof(buf), 0);
                if (n <= 0) {
                    closesocket(g_clients[i]);
                    g_clients[i] = SOCK_INVALID;
                    g_client_cnt--;
                }
            }
        }
    }
}

#ifdef _WIN32
static DWORD WINAPI server_thread_proc(LPVOID param) {
    (void)param;
    server_loop();
    return 0;
}
#else
#include <pthread.h>
static void *server_thread_proc(void *param) {
    (void)param;
    server_loop();
    return NULL;
}
#endif

void server_start(SimState *s, int port) {
    g_state = s;
    g_port = port;

    /* 初始化客户端列表 */
    int i;
    for (i = 0; i < MAX_CLIENTS; i++) g_clients[i] = SOCK_INVALID;
    g_client_cnt = 0;

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return;
    }
#endif

    g_listen = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen == SOCK_INVALID) {
        printf("Failed to create socket\n");
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(g_listen, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("Bind failed on port %d\n", port);
        closesocket(g_listen);
        g_listen = SOCK_INVALID;
        return;
    }

    listen(g_listen, 5);

#ifdef _WIN32
    {
        unsigned long mode = 1;
        ioctlsocket(g_listen, FIONBIO, &mode);
    }
#else
    {
        int flags = fcntl(g_listen, F_GETFL, 0);
        fcntl(g_listen, F_SETFL, flags | O_NONBLOCK);
    }
#endif

    g_running = 1;
    printf("WebSocket server started on ws://localhost:%d/ws\n", port);

#ifdef _WIN32
    CreateThread(NULL, 0, server_thread_proc, NULL, 0, NULL);
#else
    pthread_t tid;
    pthread_create(&tid, NULL, server_thread_proc, NULL);
    pthread_detach(tid);
#endif
}

void server_stop(void) {
    g_running = 0;
    if (g_listen != SOCK_INVALID) {
        closesocket(g_listen);
        g_listen = SOCK_INVALID;
    }
    int i;
    for (i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i] != SOCK_INVALID) {
            closesocket(g_clients[i]);
            g_clients[i] = SOCK_INVALID;
        }
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

#include "datatypes.h"
#include "simcore.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

int main(int argc, char *argv[]) {
    int item_count = 30;
    int seed = 42;
    int server_mode = 0;
    int ws_port = 8080;

    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--server") == 0 || strcmp(argv[i], "-s") == 0) {
            server_mode = 1;
        } else if (strcmp(argv[i], "--port") == 0 || strcmp(argv[i], "-p") == 0) {
            if (i + 1 < argc) ws_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("智能仓储入库上架仿真程序 v2.0\n\n");
            printf("用法: warehouse_sim [选项] [货物数量] [随机种子]\n\n");
            printf("选项:\n");
            printf("  -s, --server    启动WebSocket服务器模式\n");
            printf("  -p, --port N    指定端口 (默认8080)\n");
            printf("  -h, --help      显示帮助\n\n");
            printf("示例:\n");
            printf("  warehouse_sim 50 123         终端模式, 50货物, 种子123\n");
            printf("  warehouse_sim -s -p 9000     服务器模式, 端口9000\n");
            printf("\n按回车键退出...");
            getchar();
            return 0;
        } else if (argv[i][0] != '-') {
            /* 位置参数 */
            if (item_count == 30 && atoi(argv[i]) > 0) {
                item_count = atoi(argv[i]);
            } else {
                seed = atoi(argv[i]);
            }
        }
    }

    /* 后处理位置参数 (第一个非选项数字=货物数, 第二个=种子) */
    int pos_args = 0;
    for (i = 1; i < argc; i++) {
        if (argv[i][0] != '-') {
            int val = atoi(argv[i]);
            if (val > 0 || argv[i][0] == '0') {
                if (pos_args == 0) item_count = val;
                else if (pos_args == 1) seed = val;
                pos_args++;
            }
        }
    }

    if (item_count < 1) item_count = 1;
    if (item_count > MAX_ITEMS) item_count = MAX_ITEMS;

    printf("========================================\n");
    printf("   智能仓储入库上架仿真程序 v2.0\n");
    printf("========================================\n");
    printf("参数: 货物数=%d, 随机种子=%d, 模式=%s\n",
           item_count, seed, server_mode ? "服务器" : "终端");
    printf("\n");

    SimState state;
    sim_init(&state, item_count, seed);

    if (server_mode) {
        sim_run_with_server(&state, ws_port);
    } else {
        sim_run(&state);
    }

    printf("\n仿真结束\n");
    printf("按回车键退出...");
    getchar();
    return 0;
}

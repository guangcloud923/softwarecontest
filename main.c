/*
 * 智能仓储入库上架仿真程序
 * 入口: 支持输入仿真参数，一键运行
 *
 * 编译: gcc main.c simcore.c map.c conveyor.c agv.c robot.c constraint.c stats.c -o warehouse_sim
 * 运行: warehouse_sim [货物数量] [随机种子]
 */

#include "datatypes.h"
#include "simcore.h"
#include "stats.h"

int main(int argc, char *argv[]) {
    int cargo_count = 30;   /* 默认30个货物 */
    int seed = 42;          /* 默认种子 */

    if (argc > 1) {
        cargo_count = atoi(argv[1]);
        if (cargo_count < 1) cargo_count = 1;
        if (cargo_count > MAX_CARGOS) cargo_count = MAX_CARGOS;
    }
    if (argc > 2) {
        seed = atoi(argv[2]);
    }

    printf("========================================\n");
    printf("   智能仓储入库上架仿真程序 v1.0\n");
    printf("========================================\n");
    printf("参数: 货物数=%d, 随机种子=%d\n", cargo_count, seed);
    printf("\n");

    SimState state;
    sim_init(&state, cargo_count, seed);
    sim_run(&state);
    sim_print_score(&state);

    printf("\n仿真结束，按任意键退出...\n");
    system("pause");
    return 0;
}

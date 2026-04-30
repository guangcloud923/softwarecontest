贾浩然爱我
运行方式：
1 终端模式（命令行输出）                                                                           
   
  make clean && make                                                                               
  ./warehouse_sim 30 42                                                                            
  参数：warehouse_sim [货物数量] [随机种子]                                                        
                                                                                                   
 2服务器模式（WebSocket + 可视化）                                                                 
                                                                                                   
  make clean && make                                                                               
  ./warehouse_sim -s -p 8080 30 42                                                                 
  然后浏览器打开 http://localhost:8080，页面自动通过 WebSocket 连接并展示沙盘动画。                
                                                                                                   
  如果只想快速测试：                                                                               
  make run          # 终端模式，30货物，种子42                                                     
  make run-server   # 服务器模式，端口8080                               

为什么才38分。。。
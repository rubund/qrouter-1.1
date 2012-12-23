/*--------------------------------------------------------------*/
/* maze.h -- details of maze router                            	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on work of Steve	*/
/* Beccue.							*/
/*--------------------------------------------------------------*/

#ifndef MAZE_H

int     set_node_to_net(NODE node, int newnet, POINT *pushlist, SEG bbox);
void    ripup_net(NET net, u_char restore);
int     eval_pt(int x, int y, int lay, int thisindex, u_char stage);
int     commit_proute(ROUTE rt, u_char stage);
int     writeback_route(ROUTE rt);
int     writeback_all_routes(NET net);
NETLIST find_colliding(NET net);

extern int    CurrentLay;
extern int    NPX, NPY;

#define MAZE_H
#endif 

/* end of maze.h */

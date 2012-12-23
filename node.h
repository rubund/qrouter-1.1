/*--------------------------------------------------------------*/
/* node.h -- general purpose autorouter                      	*/
/*--------------------------------------------------------------*/
/*                Personal work of Steve Beccue                 */
/*            Copyright (C) 2003 - All Rights Reserved          */
/*--------------------------------------------------------------*/

#ifndef NODE_H

#define SRCNETNUM        1
#define TARGNETNUM       2
#define MIN_NET_NUMBER   3

void create_netorder( void );
void create_netlist( void );
void print_nodes( char *filename );
void print_nlnets( char *filename );
void create_obstructions_from_nodes( void );
void create_obstructions_from_gates( void );
void print_obs( char *filename );
void print_obs2( char *filename );
int isconnected( NODE node1, NODE node2 );
int isconnectedrecurse( NODE node1, NODE node2 );

#define NODE_H
#endif 


/* end of node.h */


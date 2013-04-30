/*--------------------------------------------------------------*/
/* maze.c -- general purpose maze router routines.		*/
/*								*/
/* This file contains the main cost evaluation routine, 	*/
/* the route segment generator, and a routine to search		*/
/* the database for all parts of a routed network.		*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on code by Steve	*/
/* Beccue							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define  MAZE

#include "qrouter.h"
#include "config.h"
#include "node.h"
#include "maze.h"

extern int TotalRoutes;

/*--------------------------------------------------------------*/
/* set_node_to_net() ---					*/
/*								*/
/* Change the Obs2[][] flag values to "newflags" for all tap	*/
/* positions of route terminal "node".  Then follow all routes	*/
/* connected to "node", updating their positions.  Where those	*/
/* routes connect to other nodes, repeat recursively.		*/
/*								*/
/* Return value is 1 if at least one terminal of the node	*/
/* is already marked as PR_SOURCE, indicating that the node	*/
/* has already been routed.  Otherwise, the return value is	*/
/* zero of no error occured, and -1 if any point was found to	*/
/* be unoccupied by any net, which should not happen.		*/
/*								*/
/* If "bbox" is non-null, record the grid extents of the node	*/
/* in the x1, x2, y1, y2 values					*/
/*								*/
/* If "stage" is 1 (rip-up and reroute), then don't let an	*/
/* existing route prevent us from adding terminals.  However,	*/
/* the area will be first checked for any part of the terminal	*/
/* that is routable, only resorting to overwriting colliding	*/
/* nets if there are no other available taps.  Defcon stage 3	*/
/* indicates desperation due to a complete lack of routable	*/
/* taps.  This happens if, for example, a port is offset from	*/
/* the routing grid and tightly boxed in by obstructions.  In	*/
/* such case, we allow routing on an obstruction, but flag the	*/
/* point.  In the output stage, the stub route information will	*/
/* be used to reposition the contact on the port and away from	*/
/* the obstruction.						*/
/*								*/
/* If we completely fail to find a tap point under any		*/
/* condition, then return -2.  This is a fatal error;  there	*/
/* will be no way to route the net.				*/
/*--------------------------------------------------------------*/

int set_node_to_net(NODE node, int newflags, POINT *pushlist, SEG bbox, u_char stage)
{
    int x, y, lay, k, obsnet = 0;
    int result = 0;
    u_char found_one = (u_char)0;
    POINT gpoint;
    DPOINT ntap;
    ROUTE rt;
    SEG seg;
    NODE n2;
    PROUTE *Pr;

    /* Process tap points of the node */

    for (ntap = node->taps; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;
       Pr = &Obs2[lay][OGRID(x, y, lay)];
       if ((Pr->flags & (newflags | PR_COST)) == PR_COST)
	  return -1;	// This should not happen.

       if (Pr->flags & PR_SOURCE) {
	  result = 1;				// Node is already connected!
       }
       else if (((Pr->prdata.net == node->netnum) || (stage == (u_char)2)) &&
			!(Pr->flags & newflags)) {

	  // Do the source and dest nodes need to be marked routable?
	  Pr->flags |= (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);

	  // If we got here, we're on the rip-up stage, and there
	  // is an existing route completely blocking the terminal.
	  // So we will route over it and flag it as a collision.
	  if (Pr->prdata.net != node->netnum) Pr->flags |= PR_CONFLICT;

	  Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     gpoint = (POINT)malloc(sizeof(struct point_));
	     gpoint->x1 = x;
	     gpoint->y1 = y;
	     gpoint->layer = lay;
	     gpoint->next = *pushlist;
	     *pushlist = gpoint;
	  }
	  found_one = (u_char)1;

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
       else if (Pr->prdata.net < Numnets) obsnet++;
    }

    // Do the same for point in the halo around the tap, but only if
    // they have been attached to the net during a past routing run.

    for (ntap = node->extend; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;

       // Don't process extended areas if they coincide with other nodes.

       if (Nodeloc[lay][OGRID(x, y, lay)] != (NODE)NULL &&
		Nodeloc[lay][OGRID(x, y, lay)] != node)
	  continue;

       Pr = &Obs2[lay][OGRID(x, y, lay)];
       if (Pr->flags & PR_SOURCE) {
	  result = 1;				// Node is already connected!
       }
       else if ( !(Pr->flags & newflags) &&
		((Pr->prdata.net == node->netnum) ||
		(stage == (u_char)2 && Pr->prdata.net < Numnets) ||
		(stage == (u_char)3))) {

	  if (Pr->prdata.net != node->netnum) Pr->flags |= PR_CONFLICT;
	  Pr->flags |= (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);
	  Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     gpoint = (POINT)malloc(sizeof(struct point_));
	     gpoint->x1 = x;
	     gpoint->y1 = y;
	     gpoint->layer = lay;
	     gpoint->next = *pushlist;
	     *pushlist = gpoint;
	  }
	  found_one = (u_char)1;

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
       else if (Pr->prdata.net < Numnets) obsnet++;
    }

    // That which is already routed (routes should be attached to source
    // nodes) is routable by definition. . .

    for (rt = node->routes; rt; rt = rt->next) {
       if (rt->segments && rt->node) {
	  for (seg = rt->segments; seg; seg = seg->next) {
	     lay = seg->layer;
	     x = seg->x1;
	     y = seg->y1;
	     while (1) {
		Pr = &Obs2[lay][OGRID(x, y, lay)];
		Pr->flags |= (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);
		// Conflicts should not happen (check for this?)
		// if (Pr->prdata.net != node->netnum) Pr->flags |= PR_CONFLICT;
		Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

		// push this point on the stack to process

		if (pushlist != NULL) {
	  	   gpoint = (POINT)malloc(sizeof(struct point_));
	  	   gpoint->x1 = x;
	  	   gpoint->y1 = y;
	  	   gpoint->layer = lay;
	  	   gpoint->next = *pushlist;
	 	   *pushlist = gpoint;
		}
		found_one = (u_char)1;

		// record extents
		if (bbox) {
		   if (x < bbox->x1) bbox->x1 = x;
		   if (x > bbox->x2) bbox->x2 = x;
		   if (y < bbox->y1) bbox->y1 = y;
		   if (y > bbox->y2) bbox->y2 = y;
		}

		// If we found another node connected to the route,
		// then process it, too.

		n2 = Nodeloc[lay][OGRID(x, y, lay)];
		if ((n2 != (NODE)NULL) && (n2 != node)) {
		   result = set_node_to_net(n2, newflags, pushlist, bbox, stage);
		}

		// Move to next grid position in segment
		if (x == seg->x2 && y == seg->y2) break;
		if (seg->x2 > seg->x1) x++;
		else if (seg->x2 < seg->x1) x--;
		if (seg->y2 > seg->y1) y++;
		else if (seg->y2 < seg->y1) y--;
	     }
	  }
       }
    }

    // In the case that no valid tap points were found,	if we're on the
    // rip-up and reroute section, try again, ignoring existing routes that
    // are in the way of the tap point.  If that fails, then we will
    // route over obstructions and shift the contact when committing the
    // route solution.  And if that fails, we're basically hosed.
    //
    // Make sure to check for the case that the tap point is simply not
    // reachable from any grid point, in the first stage, so we don't
    // wait until the rip-up and reroute stage to route them.

    if ((result == 0) && (found_one == (u_char)0)) {
       if (stage == (u_char)1)
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)2);
       else if (stage == (u_char)2)
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)3);
       else if ((stage == (u_char)0) && (obsnet == 0))
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)3);
       else
	  return -2;
    }
    return result;
}

/*--------------------------------------------------------------*/
/* Find nets that are colliding with the given net "net", and	*/
/* create and return a list of them.				*/
/*--------------------------------------------------------------*/

NETLIST find_colliding(NET net)
{
   NETLIST nl = (NETLIST)NULL, cnl;
   NODELIST ndl;
   NODE node;
   NET  fnet;
   ROUTE rt;
   SEG seg;
   int lay, x, y, orignet;

   /* Scan the routed points for recorded collisions.	*/

   for (ndl = net->netnodes; ndl; ndl = ndl->next) {
      node = ndl->node;
      for (rt = node->routes; rt; rt = rt->next) {
         if (rt->segments && rt->node) {
	    for (seg = rt->segments; seg; seg = seg->next) {
	       lay = seg->layer;
	       x = seg->x1;
	       y = seg->y1;

	       // The following skips over vias, which is okay, since
	       // those positions are either covered by segments, or
	       // are terminals of the net.

	       while (1) {
	          orignet = Obs[lay][OGRID(x, y, lay)];

	          if (orignet != net->netnum) {

	             /* Route collision.  Save this net if it is	*/
	             /* not already in the list of colliding nets.	*/

	             for (cnl = nl; cnl; cnl = cnl->next) {
		        if (cnl->net->netnum == orignet)
		           break;
	             }
	             if (cnl == NULL) {
		        cnl = (NETLIST)malloc(sizeof(struct netlist_));
		        for (fnet = Nlnets; fnet; fnet = fnet->next) {
		           if (fnet->netnum == orignet) {
		              cnl->net = fnet;
		              cnl->next = nl;
		              nl = cnl;
			      break;
			   }
			}
		     }
		  }
	          if ((x == seg->x2) && (y == seg->y2)) break;
		  if (x < seg->x2) x++;
		  else if (x > seg->x2) x--;
		  if (y < seg->y2) y++;
		  else if (y > seg->y2) y--;
	       }
	    }
	 }
      }
   }

   /* Diagnostic */

   if (nl != NULL) {
      fprintf(stderr, "Best route of %s collides with nets: ",
		net->netname);
      for (cnl = nl; cnl; cnl = cnl->next) {
         fprintf(stderr, "%s ", cnl->net->netname);
      }
      fprintf(stderr, "\n");
   }

   return nl;
}

/*--------------------------------------------------------------*/
/* ripup_net ---						*/
/*								*/
/* Rip up the entire network located at position x, y, lay.	*/
/*								*/
/* If argument "restore" is TRUE, then at each node, restore	*/
/* the crossover cost by attaching the node back to the Nodeloc	*/
/* array.							*/
/*--------------------------------------------------------------*/

u_char ripup_net(NET net, u_char restore)
{
   int thisnet, oldnet, x, y, lay, dir;
   NODE node;
   NODELIST nl;
   ROUTE rt;
   SEG seg;
   DPOINT ntap;

   thisnet = net->netnum;

   for (nl = net->netnodes; nl; nl = nl->next) {
      node = nl->node;

      for (rt = node->routes; rt; rt = rt->next) {
         if (rt->segments && rt->node) {
	    for (seg = rt->segments; seg; seg = seg->next) {
	       lay = seg->layer;
	       x = seg->x1;
	       y = seg->y1;
	       while (1) {
	          oldnet = Obs[lay][OGRID(x, y, lay)] & ~PINOBSTRUCTMASK;
	          if ((oldnet > 0) && (oldnet < Numnets)) {
	             if (oldnet != thisnet) {
		        fprintf(stderr, "Error: position %d %d layer %d has net "
				"%d not %d!\n", x, y, lay, oldnet, thisnet);
		        return FALSE;	// Something went wrong
	             }

	             // Reset the net number to zero along this route for
	             // every point that is not a node tap.  Points that
		     // were routed over obstructions to reach off-grid
		     // taps are returned to obstructions.

	             if (Nodesav[lay][OGRID(x, y, lay)] == (NODE)NULL) {
			dir = Obs[lay][OGRID(x, y, lay)] & PINOBSTRUCTMASK;
			if (dir == 0)
		           Obs[lay][OGRID(x, y, lay)] = 0;
			else
		           Obs[lay][OGRID(x, y, lay)] = (Numnets + 1) | dir;
		     }
		  }

		  // This break condition misses via ends, but those are
		  // terminals and don't get ripped out.

		  if ((x == seg->x2) && (y == seg->y2)) break;

		  if (x < seg->x2) x++;
		  else if (x > seg->x2) x--;
		  if (y < seg->y2) y++;
		  else if (y > seg->y2) y--;
	       }
	    }
	 }
      }

      // For each net node tap, restore the node pointer on Nodeloc so
      // that crossover costs are again applied to routes over this
      // node tap.

      if (restore != 0) {
	 for (ntap = node->taps; ntap; ntap = ntap->next) {
	    lay = ntap->layer;
	    x = ntap->gridx;
	    y = ntap->gridy;
	    Nodeloc[lay][OGRID(x, y, lay)] = Nodesav[lay][OGRID(x, y, lay)];
	 }
      }
   }

   /* Remove all routing information from this node */

   for (nl = net->netnodes; nl; nl = nl->next) {
      node = nl->node;
      while (node->routes) {
	 rt = node->routes;
	 node->routes = rt->next;
	 while (rt->segments) {
	    seg = rt->segments->next;
	    free(rt->segments);
	    rt->segments = seg;
         }
	 free(rt);
      }
   }

   return TRUE;
}

/*--------------------------------------------------------------*/
/* eval_pt - evaluate cost to get from given point to		*/
/*	current point.  Current point is passed in "ept", and	*/
/* 	the direction from the new point to the current point	*/
/*	is indicated by "flags".				*/
/*								*/
/*	ONLY consider the cost of the single step itself.	*/
/*								*/
/*      If "stage" is nonzero, then this is a second stage	*/
/*	routing, where we should consider other nets to be a	*/
/*	high cost to short to, rather than a blockage.  This	*/
/* 	will allow us to finish the route, but with a minimum	*/
/*	number of collisions with other nets.  Then, we rip up	*/
/*	those nets, add them to the "failed" stack, and re-	*/
/*	route this one.						*/
/*								*/
/*  ARGS: none							*/
/*  RETURNS: 1 if node needs to be (re)processed, 0 if not.	*/
/*  SIDE EFFECTS: none (get this right or else)			*/
/*--------------------------------------------------------------*/

int eval_pt(GRIDP *ept, u_char flags, u_char stage)
{
    int thiscost = 0;
    NODE node;
    NETLIST nl;
    PROUTE *Pr, *Pt;
    GRIDP newpt;

    newpt = *ept;

    switch (flags) {
       case PR_PRED_N:
	  newpt.y--;
	  break;
       case PR_PRED_S:
	  newpt.y++;
	  break;
       case PR_PRED_E:
	  newpt.x--;
	  break;
       case PR_PRED_W:
	  newpt.x++;
	  break;
       case PR_PRED_U:
	  newpt.lay--;
	  break;
       case PR_PRED_D:
	  newpt.lay++;
	  break;
    }

    Pr = &Obs2[newpt.lay][OGRID(newpt.x, newpt.y, newpt.lay)];

    if (!(Pr->flags & (PR_COST | PR_SOURCE))) {
       // 2nd stage allows routes to cross existing routes
       if (stage && (Pr->prdata.net < Numnets)) {
	  if (Nodesav[newpt.lay][OGRID(newpt.x, newpt.y, newpt.lay)] != NULL)
	     return 0;			// But cannot route over terminals!

	  // Is net k in the "noripup" list?  If so, don't route it */

	  for (nl = CurNet->noripup; nl; nl = nl->next) {
	     if (nl->net->netnum == Pr->prdata.net)
		return 0;
	  }

	  // In case of a collision, we change the grid point to be routable
	  // but flag it as a point of collision so we can later see what
	  // were the net numbers of the interfering routes by cross-referencing
	  // the Obs[][] array.

	  Pr->flags |= (PR_CONFLICT | PR_COST);
	  Pr->prdata.cost = MAXRT;
	  thiscost = ConflictCost;
       }
       else
          return 0;		// Position is not routeable
    }

    // Compute the cost to step from the current point to the new point.
    // "BlockCost" is used if the node has only one point to connect to,
    // so that routing over it could block it entirely.

    if (newpt.lay > 0) {
	if ((node = Nodeloc[newpt.lay - 1][OGRID(newpt.x, newpt.y, newpt.lay - 1)])
			!= (NODE)NULL) {
	    Pt = &Obs2[newpt.lay - 1][OGRID(newpt.x, newpt.y, newpt.lay - 1)];
	    if (!(Pt->flags & PR_TARGET) && !(Pt->flags & PR_SOURCE)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else
	           thiscost += XverCost;	// Cross-under cost
	    }
	}
    }
    if (newpt.lay < Num_layers - 1) {
	if ((node = Nodeloc[newpt.lay + 1][OGRID(newpt.x, newpt.y, newpt.lay + 1)])
			!= (NODE)NULL) {
	    Pt = &Obs2[newpt.lay + 1][OGRID(newpt.x, newpt.y, newpt.lay + 1)];
	    if (!(Pt->flags & PR_TARGET) && !(Pt->flags & PR_SOURCE)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else
	           thiscost += XverCost;	// Cross-over cost
	    }
	}
    }
    if (ept->lay != newpt.lay) thiscost += ViaCost;
    if (ept->x != newpt.x) thiscost += (Vert[newpt.lay] * JogCost +
			(1 - Vert[newpt.lay]) * SegCost);
    if (ept->y != newpt.y) thiscost += (Vert[newpt.lay] * SegCost +
			(1 - Vert[newpt.lay]) * JogCost);

    // Add the cost to the cost of the original position
    thiscost += ept->cost;
   
    // Replace node information if cost is minimum

    if (Pr->flags & PR_CONFLICT)
       thiscost += ConflictCost;	// For 2nd stage routes

    if (thiscost < Pr->prdata.cost) {
       Pr->flags &= ~PR_PRED_DMASK;
       Pr->flags |= flags;
       Pr->prdata.cost = thiscost;
       Pr->flags &= ~PR_PROCESSED;	// Need to reprocess this node

       if (Verbose > 0) {
	  fprintf(stdout, "New cost %d at (%d %d %d)\n", thiscost,
		newpt.x, newpt.y, newpt.lay);
       }
       return 1;
    }
    return 0;	// New position did not get a lower cost

} /* eval_pt() */


/*--------------------------------------------------------------*/
/* commit_proute - turn the potential route into an actual	*/
/*		route by generating the route segments		*/
/*								*/
/*  ARGS:   route structure to fill;  "stage" is 1 if we're on	*/
/*	    second stage routing, in which case we fill the	*/
/*	    route structure but don't modify the Obs array.	*/
/*								*/
/*  RETURNS: 							*/
/*  SIDE EFFECTS: Obs update, RT llseg added			*/
/*--------------------------------------------------------------*/

int commit_proute(ROUTE rt, GRIDP *ept, u_char stage)
{
   SEG  seg, lseg;
   int  i, j, k;
   int  x, y;
   int  dx, dy, dl;
   NODE n1;
   u_int netnum, netobs1, netobs2, dir1, dir2;
   u_char first = (u_char)1;
   u_char dmask;
   u_char pflags, p2flags;
   PROUTE *Pr;
   DPOINT dp;
   POINT newlr, newlr2, lrtop, lrend, lrnext, lrcur, lrprev;

   fflush(stdout);
   fprintf(stderr, "\nCommit: TotalRoutes = %d\n", TotalRoutes);

   n1 = rt->node;
   netnum = rt->netnum;

   Pr = &Obs2[ept->lay][OGRID(ept->x, ept->y, ept->lay)];
   if (!(Pr->flags & PR_COST)) {
      fprintf(stderr, "commit_proute(): impossible - terminal is not routable!\n");
      return FALSE;
   }

   // Generate an indexed route, recording the series of predecessors and their
   // positions.

   lrtop = (POINT)malloc(sizeof(struct point_));
   lrtop->x1 = ept->x;
   lrtop->y1 = ept->y;
   lrtop->layer = ept->lay;
   lrtop->next = NULL;
   lrend = lrtop;

   while (1) {

      Pr = &Obs2[lrend->layer][OGRID(lrend->x1, lrend->y1, lrend->layer)];
      dmask = Pr->flags & PR_PRED_DMASK;
      if (dmask == PR_PRED_NONE) break;

      newlr = (POINT)malloc(sizeof(struct point_));
      newlr->x1 = lrend->x1;
      newlr->y1 = lrend->y1;
      newlr->layer = lrend->layer;
      lrend->next = newlr;
      newlr->next = NULL;

      switch (dmask) {
         case PR_PRED_N:
	    (newlr->y1)++;
	    break;
         case PR_PRED_S:
	    (newlr->y1)--;
	    break;
         case PR_PRED_E:
	    (newlr->x1)++;
	    break;
         case PR_PRED_W:
	    (newlr->x1)--;
	    break;
         case PR_PRED_U:
	    (newlr->layer)++;
	    break;
         case PR_PRED_D:
	    (newlr->layer)--;
	    break;
      }
      lrend = newlr;
   }
   lrend = lrtop;

   // TEST:  Walk through the solution, and look for stacked vias.  When
   // found, look for an alternative path that avoids the stack.

   if (StackedContacts < (Num_layers - 1)) {
      POINT lrppre;
      POINT a, b;
      PROUTE *pri, *pri2;
      int stacks = 1, stackheight;
      int cx, cy, cl;
      int mincost, minx, miny, ci, ci2;

      while (stacks != 0) {	// Keep doing until all illegal stacks are gone
	 stacks = 0;
	 lrcur = lrend;
	 lrprev = lrend->next;

	 while (lrprev != NULL) {
	    lrppre = lrprev->next;
	    if (lrppre == NULL) break;
	    stackheight = 0;
	    a = lrcur;
	    b = lrprev;
	    while (a->layer != b->layer) {
	       stackheight++;
	       a = b;
	       b = a->next;
	       if (b == NULL) break;
	    }
	    if (stackheight > StackedContacts) {	// Illegal stack found
	       stacks++;

	       // Try to move the second contact in the path
	       cx = lrprev->x1;
	       cy = lrprev->y1;
	       cl = lrprev->layer;
	       mincost = MAXRT;
	       dl = lrppre->layer;

	       // Check all four positions around the contact for the
	       // lowest cost, and make sure the position below that
	       // is available.
	       dx = cx + 1;	// Check to the right
	       pri = &Obs2[cl][OGRID(dx, cy, cl)];
	       pflags = pri->flags;
	       if (pflags & PR_COST) {
		  pflags &= ~PR_COST;
		  if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	             pri2 = &Obs2[dl][OGRID(dx, cy, dl)];
		     p2flags = pri2->flags;
		     if (p2flags & PR_COST) {
			p2flags &= ~PR_COST;
		        if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri2->prdata.cost < MAXRT) {
		           mincost = pri->prdata.cost;
		           minx = dx;
		           miny = cy;
			}
		     }
		  }
	       }
	       dx = cx - 1;	// Check to the left
	       pri = &Obs2[cl][OGRID(dx, cy, cl)];
	       pflags = pri->flags;
	       if (pflags & PR_COST) {
		  pflags &= ~PR_COST;
		  if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	             pri2 = &Obs2[dl][OGRID(dx, cy, dl)];
		     p2flags = pri2->flags;
		     if (p2flags & PR_COST) {
			p2flags &= ~PR_COST;
		        if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri2->prdata.cost < MAXRT) {
		           mincost = pri->prdata.cost;
		           minx = dx;
		           miny = cy;
			}
		     }
		  }
	       }

	       dy = cy + 1;	// Check up
	       pri = &Obs2[cl][OGRID(cx, dy, cl)];
	       pflags = pri->flags;
	       if (pflags & PR_COST) {
		  pflags &= ~PR_COST;
		  if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	             pri2 = &Obs2[dl][OGRID(cx, dy, dl)];
		     p2flags = pri2->flags;
		     if (p2flags & PR_COST) {
			p2flags &= ~PR_COST;
		        if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri2->prdata.cost < MAXRT) {
		           mincost = pri->prdata.cost;
		           minx = cx;
		           miny = dy;
			}
		     }
		  }
	       }

	       dy = cy - 1;	// Check down
	       pri = &Obs2[cl][OGRID(cx, dy, cl)];
	       pflags = pri->flags;
	       if (pflags & PR_COST) {
		  pflags &= ~PR_COST;
		  if (pflags & PR_PRED_DMASK != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	             pri2 = &Obs2[dl][OGRID(cx, dy, dl)];
		     p2flags = pri2->flags;
		     if (p2flags & PR_COST) {
		        p2flags &= ~PR_COST;
		        if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri2->prdata.cost < MAXRT) {
		           mincost = pri->prdata.cost;
		           minx = cx;
		           miny = dy;
			}
		     }
		  }
	       }

	       // Was there an available route?  If so, modify
	       // records to route through this alternate path.  If not,
	       // then try to move the first contact instead.

	       if (mincost < MAXRT) {
	          pri = &Obs2[cl][OGRID(minx, miny, cl)];

		  newlr = (POINT)malloc(sizeof(struct point_));
		  newlr->x1 = minx;
		  newlr->y1 = miny;
		  newlr->layer = cl;

	          pri2 = &Obs2[dl][OGRID(minx, miny, dl)];

		  newlr2 = (POINT)malloc(sizeof(struct point_));
		  newlr2->x1 = minx;
		  newlr2->y1 = miny;
		  newlr2->layer = dl;

		  lrprev->next = newlr;
		  newlr->next = newlr2;

		  // Check if point at pri2 is equal to position of
		  // lrppre->next.  If so, bypass lrppre.

		  if (lrnext = lrppre->next) {
		     if (lrnext->x1 == minx && lrnext->y1 == miny &&
				lrnext->layer == dl) {
			newlr->next = lrnext;
			free(lrppre);
			free(newlr2);
		     }
		     else
		        newlr2->next = lrppre;
		  }
		  else
		     newlr2->next = lrppre;
	       }
	       else {

		  // If we couldn't offset lrprev position, then try
		  // offsetting lrcur.

	          cx = lrcur->x1;
	          cy = lrcur->y1;
	          cl = lrcur->layer;
	          mincost = MAXRT;
	          dl = lrprev->layer;

	          dx = cx + 1;	// Check to the right
	          pri = &Obs2[cl][OGRID(dx, cy, cl)];
	          pflags = pri->flags;
		  if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	                pri2 = &Obs2[dl][OGRID(dx, cy, dl)];
		        p2flags = pri2->flags;
			if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = pri->prdata.cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		     }
	          }

	          dx = cx - 1;	// Check to the left
	          pri = &Obs2[cl][OGRID(dx, cy, cl)];
	          pflags = pri->flags;
		  if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	                pri2 = &Obs2[dl][OGRID(dx, cy, dl)];
		        p2flags = pri2->flags;
			if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = pri->prdata.cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		     }
	          }

	          dy = cy + 1;	// Check up
	          pri = &Obs2[cl][OGRID(cx, dy, cl)];
	          pflags = pri->flags;
		  if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	                pri2 = &Obs2[dl][OGRID(cx, dy, dl)];
		        p2flags = pri2->flags;
			if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = pri->prdata.cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		     }
	          }

	          dy = cy - 1;	// Check down
	          pri = &Obs2[cl][OGRID(cx, dy, cl)];
	          pflags = pri->flags;
		  if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
				pri->prdata.cost < mincost) {
	                pri2 = &Obs2[dl][OGRID(cx, dy, dl)];
		        p2flags = pri2->flags;
			if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = pri->prdata.cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		     }
	          }

		  if (mincost < MAXRT) {
	             pri = &Obs2[cl][OGRID(minx, miny, cl)];

		     newlr = (POINT)malloc(sizeof(struct point_));
		     newlr->x1 = minx;
		     newlr->y1 = miny;
		     newlr->layer = cl;

	             pri2 = &Obs2[dl][OGRID(minx, miny, dl)];

		     newlr2 = (POINT)malloc(sizeof(struct point_));
		     newlr2->x1 = minx;
		     newlr2->y1 = miny;
		     newlr2->layer = dl;

		     lrcur->next = newlr;
		     newlr->next = newlr2;

		     // Check if point at pri2 is equal to position of
		     // lrprev->next.  If so, bypass lrprev.

		     if (lrppre->x1 == minx && lrppre->y1 == miny &&
				lrppre->layer == dl) {
			newlr->next = lrppre;
			free(lrprev);
			free(newlr2);
			lrprev = lrcur;
		     }
		     else
			newlr2->next = lrprev;
		  }
		  else {
		     printf("Error:  Failed to remove stacked via at grid "
				"point %d %d!\n", lrcur->x1, lrcur->y1);
		     stacks = 0;
		  }
	       }
	    }
	    lrcur = lrprev;
	    lrprev = lrppre;
	 }
      }
   }
 
   lrend = lrtop;
   lrcur = lrtop;
   lrprev = lrcur->next;
   lseg = (SEG)NULL;

   while (1) {
      seg = (SEG)malloc(sizeof(struct seg_));
      seg->next = NULL;

      seg->segtype = (lrcur->layer == lrprev->layer) ? ST_WIRE : ST_VIA;

      seg->x1 = lrcur->x1;
      seg->y1 = lrcur->y1;

      seg->layer = MIN(lrcur->layer, lrprev->layer);

      seg->x2 = lrprev->x1;
      seg->y2 = lrprev->y1;

      dx = seg->x2 - seg->x1;
      dy = seg->y2 - seg->y1;

      // segments are in order---place final segment at end of list
      if (rt->segments == NULL)
	 rt->segments = seg;
      else
	 lseg->next = seg;

      // Continue processing predecessors as long as the direction is the same,
      // so we get a single long wire segment.  This minimizes the number of
      // segments produced.  Vias have to be handled one at a time, as we make
      // no assumptions about stacked vias.

      if (seg->segtype == ST_WIRE) {
	 while ((lrnext = lrprev->next) != NULL) {
	    lrnext = lrprev->next;
	    if (((lrnext->x1 - lrprev->x1) == dx) &&
			((lrnext->y1 - lrprev->y1) == dy) &&
			(lrnext->layer == lrprev->layer)) {
	       lrcur = lrprev;
	       lrprev = lrnext;
	       seg->x2 = lrprev->x1;
	       seg->y2 = lrprev->y1;
	    }
	    else
	       break;
	 }
      }

      if (Verbose > 0) {
         printf( "commit: index = %d, net = %d\n",
		Pr->prdata.net, netnum);

	 if (seg->segtype == ST_WIRE) {
            printf( "commit: wire layer %d, (%d,%d) to (%d,%d)\n",
		seg->layer, seg->x1, seg->y1, seg->x2, seg->y2);
	 }
	 else {
            printf( "commit: via %d to %d\n", seg->layer, seg->layer + 1);
	 }
	 fflush(stdout);
      }

      // now fill in the Obs structure with this route....

      netobs1 = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)];
      netobs2 = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)];

      dir1 = netobs1 & PINOBSTRUCTMASK;
      dir2 = netobs2 & PINOBSTRUCTMASK;

      netobs1 &= ~PINOBSTRUCTMASK;
      netobs2 &= ~PINOBSTRUCTMASK;

      // If Obs shows this position as an obstruction, then this was a port with
      // no taps in reach of a grid point.  This will be dealt with by moving
      // the via off-grid and onto the port position in emit_routes().

      if (stage == (u_char)0) {
         if (seg->segtype == ST_VIA) {
	    Obs[seg->layer][OGRID(lrcur->x1, lrcur->y1, seg->layer)] = netnum;
	    Obs[seg->layer + 1][OGRID(lrcur->x1, lrcur->y1, seg->layer + 1)] = netnum;
         }
         else if (seg->segtype == ST_WIRE) {
	    if (seg->x1 < seg->x2) {
	       for (i = seg->x1; i <= seg->x2; i++) {
	          Obs[lrcur->layer][OGRID(i, lrcur->y1, lrcur->layer)] = netnum;
	       }
	    }
	    if (seg->x1 > seg->x2) {
	       for (i = seg->x2; i <= seg->x1; i++) {
	          Obs[lrcur->layer][OGRID(i, lrcur->y1, lrcur->layer)] = netnum;
	       }
	    }
	    if (seg->y1 < seg->y2) {
	       for (i = seg->y1; i <= seg->y2; i++) {
	          Obs[lrcur->layer][OGRID(lrcur->x1, i, lrcur->layer)] = netnum;
	       }
	    }
	    if (seg->y1 > seg->y2) {
	       for (i = seg->y2; i <= seg->y1; i++) {
	          Obs[lrcur->layer][OGRID(lrcur->x1, i, lrcur->layer)] = netnum;
	       }
	    }
         }
         if (first && dir1) {
	    first = (u_char)0;
	    Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] |= dir1;
         }
      }

      // Keep stub information on obstructions that have been routed
      // over, so that in the rip-up stage, we can return them to obstructions.

      if (netobs1 > Numnets)
	  Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] |= dir1;
      if (netobs2 > Numnets)
	  Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] |= dir2;

      // An offset route end on the previous segment, if it is a via, needs
      // to carry over to this one, if it is a wire route.

      if (lseg && ((lseg->segtype & (ST_VIA | ST_OFFSET_END)) ==
			(ST_VIA | ST_OFFSET_END)))
	 if (seg->segtype != ST_VIA)
	    seg->segtype |= ST_OFFSET_START;

      // Check if the route ends are offset.  If so, add flags.  The segment
      // entries are integer grid points, so offsets need to be made when
      // the location is output.

      if (dir1 & OFFSET_TAP) {
	 seg->segtype |= ST_OFFSET_START;

	 // An offset on a via needs to be applied to the previous route
	 // segment as well, if that route is a wire.

	 if (lseg && (seg->segtype & ST_VIA) && !(lseg->segtype & ST_VIA))
	    lseg->segtype |= ST_OFFSET_END;
      }

      if (dir2 & OFFSET_TAP) seg->segtype |= ST_OFFSET_END;

      lrend = lrcur;		// Save the last route position
      lrend->x1 = lrcur->x1;
      lrend->y1 = lrcur->y1;
      lrend->layer = lrcur->layer;

      lrcur = lrprev;		// Move to the next route position
      lrcur->x1 = seg->x2;
      lrcur->y1 = seg->y2;
      lrprev = lrcur->next;

      dp = n1->taps;
      if (dp == NULL) dp = n1->extend;

      if (lrprev == NULL) {

         if (dir2 && (stage == (u_char)0)) {
	    Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] |= dir2;
         }

	 // Before returning, set *ept to the endpoint
	 // position.  This is for diagnostic purposes only.
	 ept->x = lrend->x1;
	 ept->y = lrend->y1;
	 ept->lay = lrend->layer;

	 // Clean up allocated memory for the route. . .
	 while (lrtop != NULL) {
	    lrnext = lrtop->next;
	    free(lrtop);
	    lrtop = lrnext;
	 }
	 return TRUE;
      }

      lseg = seg;	// Move to next segment position
   }

   // This block is not reachable
   return FALSE;

} /* commit_proute() */

/*------------------------------------------------------*/
/* writeback_route() ---				*/
/*							*/
/*   This routine is the last part of the routine	*/
/*   above.  It copies the net defined by the segments	*/
/*   in the route structure "rt" into the Obs array.	*/
/*   This is used only for stage 2, when the writeback	*/
/*   is not done by commit_proute because we want to	*/
/*   rip up nets first.					*/
/*------------------------------------------------------*/

int writeback_route(ROUTE rt)
{
   SEG seg;
   int  i;
   NODE n1;
   u_int netnum, dir1, dir2;
   u_char first = (u_char)1;

   netnum = rt->netnum;
   for (seg = rt->segments; seg; seg = seg->next) {

      /* Save stub route information at segment ends */
      dir1 = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] & PINOBSTRUCTMASK;
      dir2 = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] & PINOBSTRUCTMASK;

      if (seg->segtype & ST_VIA) {
	 Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] = netnum;
	 Obs[seg->layer + 1][OGRID(seg->x1, seg->y1, seg->layer + 1)] = netnum;
      }
      else if (seg->segtype & ST_WIRE) {
	 if (seg->x1 < seg->x2) {
	    for (i = seg->x1; i <= seg->x2; i++) {
	       Obs[seg->layer][OGRID(i, seg->y1, seg->layer)] = netnum;
	    }
	 }
	 if (seg->x1 > seg->x2) {
	    for (i = seg->x2; i <= seg->x1; i++) {
	       Obs[seg->layer][OGRID(i, seg->y1, seg->layer)] = netnum;
	    }
	 }
	 if (seg->y1 < seg->y2) {
	    for (i = seg->y1; i <= seg->y2; i++) {
	       Obs[seg->layer][OGRID(seg->x1, i, seg->layer)] = netnum;
	    }
	 }
	 if (seg->y1 > seg->y2) {
	    for (i = seg->y2; i <= seg->y1; i++) {
	       Obs[seg->layer][OGRID(seg->x1, i, seg->layer)] = netnum;
	    }
	 }
      }
      if (first && dir1) {
	 first = (u_char)0;
	 Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] |= dir1;
      }

      if (!seg->next && dir2) {
	 Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] |= dir2;
      }
   }
   return TRUE;
}

/*------------------------------------------------------*/
/* Writeback all routes belonging to a net		*/
/*------------------------------------------------------*/

int writeback_all_routes(NET net)
{
   ROUTE rt;
   NODELIST ndl;
   NODE node;
   int result = TRUE;

   for (ndl = net->netnodes; ndl; ndl = ndl->next) {
      node = ndl->node;
      for (rt = node->routes; rt; rt = rt->next) {
	 if (writeback_route(rt) == FALSE)
	    result = FALSE;
      }
   }
   return result;
}

/* end of maze.c */

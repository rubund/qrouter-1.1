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

int    CurrentLay;
int    NPX, NPY;

/*--------------------------------------------------------------*/
/* make_new_pr() ---						*/
/*								*/
/* Create a new "potential route" structure and fill its values	*/
/*--------------------------------------------------------------*/

int
make_new_pr(x, y, lay)
{
   int k;

   PRind++;

   // Check bounds on PRind
   if (PRind >= PRindMAX) return -1;

   /* Create new potential route structure */
   k = PRind;
   Pr[k].pred = 0;
   Pr[k].flags = 0;
   Pr[k].cost = 0;
   Pr[k].x1 = x;
   Pr[k].y1 = y;
   Pr[k].layer = lay;

   return k;
}

/*--------------------------------------------------------------*/
/* set_node_to_net() ---					*/
/*								*/
/* Change the Obs2[][] matrix values to "newnet" for all tap	*/
/* positions of route terminal "node".  Then follow all routes	*/
/* connected to "node", updating their positions.  Where those	*/
/* routes connect to other nodes, repeat recursively.		*/
/*								*/
/* Return value is 1 if at least one terminal of the node	*/
/* is already marked as "newnet", indicating that the node	*/
/* has already been routed.  Otherwise, the return value is	*/
/* zero of no error occured, and -1 if any point was found to	*/
/* be unoccupied by any net, which should not happen.		*/
/*								*/
/* If "bbox" is non-null, record the grid extents of the node	*/
/* in the x1, x2, y1, y2 values					*/
/*--------------------------------------------------------------*/

int set_node_to_net(NODE node, int newnet, POINT *pushlist, SEG bbox)
{
    int x, y, lay, k;
    int oldnet;
    int result = 0;
    POINT gpoint;
    DPOINT ntap;
    ROUTE rt;
    SEG seg;
    NODE n2;

    /* Process tap points of the node */

    for (ntap = node->taps; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;
       oldnet = Obs2[lay][OGRID(x, y, lay)];
       if (!oldnet) return -1;			// This should not happen.
       if (oldnet & SRCFLAG) {
	  result = 1;				// Node is already connected!
       }
       else if (oldnet != newnet) {
	  if (newnet == SRCNETNUM) {
	     k = make_new_pr(x, y, lay);
	     if (k < 0) return k;
             Obs2[lay][OGRID(x, y, lay)] = k | SRCFLAG;
	  }
	  else
             Obs2[lay][OGRID(x, y, lay)] = newnet;

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     gpoint = (POINT)malloc(sizeof(struct point_));
	     gpoint->x1 = x;
	     gpoint->y1 = y;
	     gpoint->layer = lay;
	     gpoint->next = *pushlist;
	     *pushlist = gpoint;
	  }

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
    }

    // Do the same for point in the halo around the tap, but only if
    // they have been attached to the net during a past routing run.

    for (ntap = node->extend; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;

       // Don't process extended areas if they coincide with
       // other nodes.

       if (Nodeloc[lay][OGRID(x, y, lay)] != (NODE)NULL &&
		Nodeloc[lay][OGRID(x, y, lay)] != node)
	  continue;

       oldnet = Obs2[lay][OGRID(x, y, lay)];
       if (oldnet & SRCFLAG) {
	  result = 1;				// Node is already connected!
       }
       else if (oldnet == node->netnum) {
	  if (newnet == SRCNETNUM) {
	     k = make_new_pr(x, y, lay);
	     if (k < 0) return k;
             Obs2[lay][OGRID(x, y, lay)] = k | SRCFLAG;
	  }
	  else
             Obs2[lay][OGRID(x, y, lay)] = newnet;

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     gpoint = (POINT)malloc(sizeof(struct point_));
	     gpoint->x1 = x;
	     gpoint->y1 = y;
	     gpoint->layer = lay;
	     gpoint->next = *pushlist;
	     *pushlist = gpoint;
	  }

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
    }

    for (rt = node->routes; rt; rt = rt->next) {
       if (rt->segments && rt->node) {
	  for (seg = rt->segments; seg; seg = seg->next) {
	     lay = seg->layer;
	     x = seg->x1;
	     y = seg->y1;
	     while (1) {
	        oldnet = Obs2[lay][OGRID(x, y, lay)];
		if (!oldnet) break;
		if ((oldnet != newnet) && !(oldnet & SRCFLAG)) {
		   if (newnet == SRCNETNUM) {
		      k = make_new_pr(x, y, lay);
		      if (k < 0) return k;
		      Obs2[lay][OGRID(x, y, lay)] = k | SRCFLAG;
		   }
		   else
		      Obs2[lay][OGRID(x, y, lay)] = newnet;

		   // push this point on the stack to process

		   if (pushlist != NULL) {
	  	      gpoint = (POINT)malloc(sizeof(struct point_));
	  	      gpoint->x1 = x;
	  	      gpoint->y1 = y;
	  	      gpoint->layer = lay;
	  	      gpoint->next = *pushlist;
	 	      *pushlist = gpoint;
		   }

		   // record extents
		   if (bbox) {
		      if (x < bbox->x1) bbox->x1 = x;
		      if (x > bbox->x2) bbox->x2 = x;
		      if (y < bbox->y1) bbox->y1 = y;
		      if (y > bbox->y2) bbox->y2 = y;
		   }

		   n2 = Nodeloc[lay][OGRID(x, y, lay)];
		   if ((n2 != (NODE)NULL) && (n2 != node)) {
		      result = set_node_to_net(n2, newnet, pushlist, bbox);
		   }
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
   int lay, x, y, k, orignet;

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

   if (nl == NULL) {
      fprintf(stderr, "Error:  Failed to find colliding nets!\n");
   }
   else {
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

void ripup_net(NET net, u_char restore)
{
   int thisnet, oldnet, x, y, lay;
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
	          if (oldnet != 0) {
	             if (oldnet != thisnet) {
		        fprintf(stderr, "Error: position %d %d layer %d has net "
				"%d not %d!\n", x, y, lay, oldnet, thisnet);
		        break;	// Something went wrong
	             }

	             // Reset the net number to zero along this route for
	             // every point that is not a node tap.

	             if (Nodesav[lay][OGRID(x, y, lay)] == (NODE)NULL)
		        Obs[lay][OGRID(x, y, lay)] = 0;
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
}

/*--------------------------------------------------------------*/
/* eval_pt - evaluate cost to get from given pt (x, y, lay) to	*/
/*	current point (NPX, NPY, CurrentLay)			*/
/*	ONLY consider the cost of the single step itself.	*/
/*	"thisindex" is the current index into the Pr array so	*/
/*	we can add the cost to the cost for the incremental	*/
/*	step.							*/
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

int eval_pt(int x, int y, int lay, int thisindex, u_char stage)
{
    u_char conflict = (u_char)0;
    int k, j, thiscost = 0;
    NODE node;
    NETLIST nl;

    k = Obs2[lay][OGRID(x, y, lay)];

    if (k && !(k & RTFLAG)) {
       if (k != TARGNETNUM) {
	  // 2nd stage allows routes to cross existing routes
	  if (stage && (k < Numnets)) {
	     if (Nodesav[lay][OGRID(x, y, lay)] != NULL)
		return 0;			// But cannot route over terminals!

	     // Is net k in the "noripup" list?  If so, don't route it */

	     for (nl = CurNet->noripup; nl; nl = nl->next) {
		if (nl->net->netnum == k)
		   return 0;
	     }
	     conflict = (u_char)PR_CONFLICT;	// Save the number of the colliding net
	     k = 0;
	     thiscost = ConflictCost;
	  }
	  else
             return 0;		// Position is not routeable
       }
       else
          k = RTFLAG;		// Use Pr[0] to hold target cost info
    }

    if (!k) {
       /* Not visited before, so create new entry */
       k = make_new_pr(x, y, lay);
       if (k < 0) return k;
       Pr[k].flags = conflict;
       Pr[k].cost = MAXRT;

       if (Verbose > 0) {
	  fprintf(stdout, "PRind = %d at (%d %d %d)\n", k, x, y, lay);
       }

       k |= RTFLAG;
       Obs2[lay][OGRID(x, y, lay)] = k;
    }

    // Compute the cost to step from (NPX, NPY, CurrentLay) to (x, y, lay)
    // "BlockCost" is used if the node has only one point to connect to,
    // so that routing over it could block it entirely.

    if (lay > 0) {
	if ((node = Nodeloc[lay - 1][OGRID(x, y, lay - 1)]) != (NODE)NULL) {
	    j = Obs2[lay - 1][OGRID(x, y, lay - 1)];
	    if ((j != TARGNETNUM) && !(j & SRCFLAG)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else
	           thiscost += XverCost;	// Cross-under cost
	    }
	}
    }
    if (lay < Num_layers - 1) {
	if ((node = Nodeloc[lay + 1][OGRID(x, y, lay + 1)]) != (NODE)NULL) {
	    j = Obs2[lay + 1][OGRID(x, y, lay + 1)];
	    if ((j != TARGNETNUM) && !(j & SRCFLAG)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else
	           thiscost += XverCost;	// Cross-over cost
	    }
	}
    }
    if (CurrentLay != lay) thiscost += ViaCost;
    if (NPX != x) thiscost += (Vert[lay] * JogCost + (1 - Vert[lay]) * SegCost);
    if (NPY != y) thiscost += (Vert[lay] * SegCost + (1 - Vert[lay]) * JogCost);

    // Add the cost to the cost of the original position
    if (thisindex) {	/* == 0 only for source node */
       thiscost += Pr[thisindex].cost;
    }
   
    // Replace node information if cost is minimum

    k &= ~RTFLAG;

    if (Pr[k].flags & PR_CONFLICT)
       thiscost += ConflictCost;	// For 2nd stage routes

    if (thiscost < Pr[k].cost) {
       Pr[k].pred = thisindex;
       Pr[k].cost = thiscost;
       Pr[k].flags &= ~PR_PROCESSED;	// Need to reprocess this node

       if (Verbose > 0) {
	  fprintf(stdout, "New cost %d at (%d %d %d)\n", thiscost, x, y, lay);
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

int commit_proute(ROUTE rt, u_char stage)
{
   SEG  seg, lseg;
   int  i, j, k;
   int  x, y;
   NODE n1;
   u_int netnum, dir1, dir2;
   u_char first = (u_char)1;
   int  dist, max, xn, yn, ll, dx, dy;
   int  layer, lay;
   int  index, prev, next;
   DPOINT dp;

   fflush(stdout);
   fprintf(stderr, "\nCommit: TotalRoutes = %d\n", TotalRoutes);

   n1 = rt->node;
   netnum = rt->netnum;

   xn = NPX; yn = NPY; ll = CurrentLay;
   index = Obs2[ll][OGRID(xn, yn, ll)];

   if (!(index & RTFLAG)) {
   fprintf(stderr, "commit_proute(): impossible - no prouter here!\n");
     print_obs2("obs2.failed");
     exit(-2);
   }

   if (Debug) {
      print_obs("obs.commit");
      print_obs2("obs2.commit");
   }

   index &= ~RTFLAG;
   prev = Pr[index].pred;
   lseg = (SEG)NULL;

   // TEST:  Walk through the solution, and look for stacked vias.  When
   // found, look for an alternative path that avoids the stack.

   if (StackedContacts < (Num_layers - 1)) {
      int tind, ppre, stacks = 1, stackheight, a, b;
      int cx, cy, cl, mincost;
      int dx, dy, dl, minx, miny, ci, ci2;

      // Fill in information for final route point
      Pr[index].x1 = xn;
      Pr[index].y1 = yn;
      Pr[index].layer = ll;

      while (stacks != 0) {
	 stacks = 0;
	 tind = index;
	 prev = Pr[tind].pred;
	 while (prev != 0) {
	    ppre = Pr[prev].pred;
	    if (ppre == 0) break;
	    stackheight = 0;
	    a = tind;
	    b = prev;
	    while (Pr[a].layer != Pr[b].layer) {
	       stackheight++;
	       a = b;
	       b = Pr[a].pred;
	       if (b == 0) break;
	    }
	    if (stackheight > StackedContacts) {
	       stacks++;

	       // Try to move the second contact in the path
	       cx = Pr[prev].x1;
	       cy = Pr[prev].y1;
	       cl = Pr[prev].layer;
	       mincost = MAXRT;
	       dl = Pr[ppre].layer;

	       // Check all four positions around the contact for the
	       // lowest cost, and make sure the position below that
	       // is available.
	       dx = cx + 1;	// Check to the right
	       ci = Obs2[cl][OGRID(dx, cy, cl)];
	       if (ci & RTFLAG) {
		  ci &= ~RTFLAG;
		  if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	             ci2 = Obs2[dl][OGRID(dx, cy, dl)];
		     if (ci2 & RTFLAG) {
			ci2 &= ~RTFLAG;
		        if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		           mincost = Pr[ci].cost;
		           minx = dx;
		           miny = cy;
			}
		     }
		  }
	       }
	       dx = cx - 1;	// Check to the left
	       ci = Obs2[cl][OGRID(dx, cy, cl)];
	       if (ci & RTFLAG) {
		  ci &= ~RTFLAG;
		  if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	             ci2 = Obs2[dl][OGRID(dx, cy, dl)];
		     if (ci2 & RTFLAG) {
			ci2 &= ~RTFLAG;
		        if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		           mincost = Pr[ci].cost;
		           minx = dx;
		           miny = cy;
			}
		     }
		  }
	       }

	       dy = cy + 1;	// Check up
	       ci = Obs2[cl][OGRID(cx, dy, cl)];
	       if (ci & RTFLAG) {
		  ci &= ~RTFLAG;
		  if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	             ci2 = Obs2[dl][OGRID(cx, dy, dl)];
		     if (ci2 & RTFLAG) {
			ci2 &= ~RTFLAG;
		        if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		           mincost = Pr[ci].cost;
		           minx = cx;
		           miny = dy;
			}
		     }
		  }
	       }

	       dy = cy - 1;	// Check down
	       ci = Obs2[cl][OGRID(cx, dy, cl)];
	       if (ci & RTFLAG) {
		  ci &= ~RTFLAG;
		  if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	             ci2 = Obs2[dl][OGRID(cx, dy, dl)];
		     if (ci2 & RTFLAG) {
		        ci2 &= ~RTFLAG;
		        if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		           mincost = Pr[ci].cost;
		           minx = cx;
		           miny = dy;
			}
		     }
		  }
	       }

	       // Was there an available route?  If so, modify "pred"
	       // records to route through this alternate path.  If not,
	       // then try to move the first contact instead.

	       if (mincost < MAXRT) {
	          ci = Obs2[cl][OGRID(minx, miny, cl)] & ~RTFLAG;
		  Pr[prev].pred = ci;
	          ci2 = Obs2[dl][OGRID(minx, miny, dl)] & ~RTFLAG;
		  Pr[ci].pred = ci2;
		  if (Pr[ppre].pred != ci2) {
		     Pr[ci2].pred = ppre;
		  }
	       }
	       else {
	          cx = Pr[tind].x1;
	          cy = Pr[tind].y1;
	          cl = Pr[tind].layer;
	          mincost = MAXRT;
	          dl = Pr[prev].layer;

	          dx = cx + 1;	// Check to the right
	          ci = Obs2[cl][OGRID(dx, cy, cl)];
		  if (ci & RTFLAG) {
		     ci &= ~RTFLAG;
		     if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	                ci2 = Obs2[dl][OGRID(dx, cy, dl)];
			if (ci2 & RTFLAG) {
			   ci2 &= ~RTFLAG;
		           if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		              mincost = Pr[ci].cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		     }
	          }

	          dx = cx - 1;	// Check to the left
	          ci = Obs2[cl][OGRID(dx, cy, cl)];
		  if (ci & RTFLAG) {
		     ci &= ~RTFLAG;
		     if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	                ci2 = Obs2[dl][OGRID(dx, cy, dl)];
			if (ci2 & RTFLAG) {
			   ci2 &= ~RTFLAG;
		           if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		              mincost = Pr[ci].cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		     }
	          }

	          dy = cy + 1;	// Check up
	          ci = Obs2[cl][OGRID(cx, dy, cl)];
		  if (ci & RTFLAG) {
		     ci &= ~RTFLAG;
		     if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	                ci2 = Obs2[dl][OGRID(cx, dy, dl)];
			if (ci2 & RTFLAG) {
			   ci2 &= ~RTFLAG;
		           if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		              mincost = Pr[ci].cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		     }
	          }

	          dy = cy - 1;	// Check down
	          ci = Obs2[cl][OGRID(cx, dy, cl)];
		  if (ci & RTFLAG) {
		     ci &= ~RTFLAG;
		     if (Pr[ci].pred != 0 && Pr[ci].cost < mincost) {
	                ci2 = Obs2[dl][OGRID(cx, dy, dl)];
			if (ci2 & RTFLAG) {
			   ci2 &= ~RTFLAG;
		           if (Pr[ci2].pred != 0 && Pr[ci2].cost < MAXRT) {
		              mincost = Pr[ci].cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		     }
	          }

		  if (mincost < MAXRT) {
	             ci = Obs2[cl][OGRID(minx, miny, cl)] & ~RTFLAG;
		     Pr[tind].pred = ci;
	             ci2 = Obs2[dl][OGRID(minx, miny, dl)] & ~RTFLAG;
		     Pr[ci].pred = ci2;
		     Pr[ci2].pred = prev;
		     if (Pr[prev].pred != ci2) {
		        Pr[ci2].pred = prev;
		     }
		  }
		  else {
		     printf("Warning:  Failed to remove stacked via!\n");
		     stacks = 0;
		  }
	       }
	    }
	    tind = prev;
	    prev = ppre;
	 }
      }
   }
 
   index &= ~RTFLAG;
   prev = Pr[index].pred;
   lseg = (SEG)NULL;

   while (1) {
      dist = 1;
      seg = (SEG)malloc(sizeof(struct seg_));
      seg->next = NULL;

      seg->segtype = (ll == Pr[prev].layer) ? ST_WIRE : ST_VIA;

      seg->x1 = xn;
      seg->y1 = yn;

      seg->layer = MIN(ll, Pr[prev].layer);

      seg->x2 = Pr[prev].x1;
      seg->y2 = Pr[prev].y1;

      dx = seg->x2 - seg->x1;
      dy = seg->y2 - seg->y1;

      // segments are in order---place final segment at end of list
      if (rt->segments == NULL)
	 rt->segments = seg;
      else
	 lseg->next = seg;
      lseg = seg;

      // Continue processing predecessors as long as the direction is the same,
      // so we get a single long wire segment.  This minimizes the number of
      // segments produced.  Vias have to be handled one at a time, as we make
      // no assumptions about stacked vias.

      if (seg->segtype == ST_WIRE) {
	 while ((next = Pr[prev].pred) != 0) {
	    next = Pr[prev].pred;
	    if (((Pr[next].x1 - Pr[prev].x1) == dx) &&
			((Pr[next].y1 - Pr[prev].y1) == dy) &&
			(Pr[next].layer == Pr[prev].layer)) {
	       index = prev;
	       prev = next;
	       seg->x2 = Pr[prev].x1;
	       seg->y2 = Pr[prev].y1;
	       dist++;
	    }
	    else
	       break;
	 }
      }

      if (Verbose > 0) {
         printf( "commit: index = %d, net = %d, predecessor = %d\n",
		index, netnum, prev);

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

      dir1 = Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] & PINOBSTRUCTMASK;
      dir2 = Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] & PINOBSTRUCTMASK;

      if (stage == (u_char)0) {
         if (seg->segtype == ST_VIA) {
	    Obs[seg->layer][OGRID(xn, yn, seg->layer)] = netnum;
	    Obs[seg->layer + 1][OGRID(xn, yn, seg->layer + 1)] = netnum;
         }
         else if (seg->segtype == ST_WIRE) {
	    if (seg->x1 < seg->x2) {
	       for (i = seg->x1; i <= seg->x2; i++) {
	          Obs[ll][OGRID(i, yn, ll)] = netnum;
	       }
	    }
	    if (seg->x1 > seg->x2) {
	       for (i = seg->x2; i <= seg->x1; i++) {
	          Obs[ll][OGRID(i, yn, ll)] = netnum;
	       }
	    }
	    if (seg->y1 < seg->y2) {
	       for (i = seg->y1; i <= seg->y2; i++) {
	          Obs[ll][OGRID(xn, i, ll)] = netnum;
	       }
	    }
	    if (seg->y1 > seg->y2) {
	       for (i = seg->y2; i <= seg->y1; i++) {
	          Obs[ll][OGRID(xn, i, ll)] = netnum;
	       }
	    }
         }
         if (first && dir1) {
	    first = (u_char)0;
	    Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] |= dir1;
         }
      }

      xn = seg->x2; yn = seg->y2; ll = Pr[prev].layer;

      index = prev;
      prev = Pr[index].pred;

      // Problem:  index should always be zero at source, but it's not.
      // So we check here if the predecessor position is equal to the source.

      dp = n1->taps;
      if (dp == NULL) dp = n1->extend;

      if ((index == 0) || (prev == 0) ||
		((xn == dp->gridx) && (yn == dp->gridy) && (ll == dp->layer))) { 

         if (dir2 && (stage == (u_char)0)) {
	    Obs[seg->layer][OGRID(seg->x2, seg->y2, seg->layer)] |= dir2;
         }
	 if (Debug) print_obs("obs.f1");

	 // Before returning, set NPX, NPY, and CurrentLay to the endpoint
	 // position.  This is for diagnostic purposes only.
	 NPX = Pr[index].x1;
	 NPY = Pr[index].y1; 
	 CurrentLay = Pr[index].layer;

	 return TRUE;
      }
   }

   // This block is not reachable

   if (Debug) {
      print_obs( "obs.f2" );
      print_obs2( "obs2.f2" );
   }
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

      if (seg->segtype == ST_VIA) {
	 Obs[seg->layer][OGRID(seg->x1, seg->y1, seg->layer)] = netnum;
	 Obs[seg->layer + 1][OGRID(seg->x1, seg->y1, seg->layer + 1)] = netnum;
      }
      else if (seg->segtype == ST_WIRE) {
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

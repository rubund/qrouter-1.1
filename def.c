/*
 * def.c --      
 *
 * This module incorporates the LEF/DEF format for standard-cell place and
 * route.
 *
 * Version 0.1 (September 26, 2003):  DEF input of designs.
 *
 * Written by Tim Edwards, Open Circuit Design
 * Modified April 2013 for use with qrouter
 *
 * It is assumed that the LEF files have been read in prior to this, and
 * layer information is already known.  The DEF file should have information
 * primarily on die are, track placement, pins, components, and nets.
 *
 * To-do: Routed nets should have their routes dropped into track obstructions,
 * and the nets should be ignored.  Currently, routed nets are parsed and the
 * routes are ignored.
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <math.h>		/* for roundf() function, if std=c99 */

#include "qrouter.h"
#include "node.h"
#include "config.h"
#include "maze.h"
#include "lef.h"

/*
 *------------------------------------------------------------
 *
 * DefAddRoutes --
 *
 *	Parse a network route statement from the DEF file,
 *	and add it to the linked list representing the route.
 *
 * Results:
 *	Returns the last token encountered.
 *
 * Side Effects:
 *	Reads from input stream;
 *	Adds information to the layout database.
 *
 *------------------------------------------------------------
 */

char *
DefAddRoutes(FILE *f, float oscale, char special)
{
    char *token;
    DSEG routeList, newRoute = NULL, routeTop = NULL;
    struct point_ refp;
    char valid = FALSE;		/* is there a valid reference point? */
    char initial = TRUE;
    struct dseg_ locarea;
    float x, y, z, w;
    int routeWidth, paintWidth, saveWidth;
    int routeLayer, paintLayer;
    LefList lefl;

    while (initial || (token = LefNextToken(f, TRUE)) != NULL)
    {
	/* Get next point, token "NEW", or via name */
	if (initial || !strcmp(token, "NEW") || !strcmp(token, "new"))
	{
	    /* initial pass is like a NEW record, but has no NEW keyword */
	    initial = FALSE;

	    /* invalidate reference point */
	    valid = FALSE;

	    token = LefNextToken(f, TRUE);
	    routeLayer = LefFindLayerNum(token);

	    if (routeLayer < 0)
	    {
		LefError("Unknown layer type \"%s\" for NEW route\n", token); 
		continue;
	    }
	    paintLayer = routeLayer;

	    if (special)
	    {
		/* SPECIALNETS has the additional width */
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%f", &w) != 1)
		{
		    LefError("Bad width in special net\n");
		    continue;
		}
		if (w != 0)
		    paintWidth = (int)roundf(w / oscale);
		else
		    paintWidth = LefGetRouteWidth(paintLayer); 
		saveWidth = paintWidth;
	    }
	    else
		paintWidth = LefGetRouteWidth(paintLayer); 
	}
	else if (*token != '(')	/* via name */
	{
	    /* A '+' or ';' record ends the route */
	    if (*token == ';' || *token == '+')
		break;

	    else if (valid == FALSE)
	    {
		LefError("Route has via name \"%s\" but no points!\n", token);
		continue;
	    }
	    lefl = LefFindLayer(token);
	    if (lefl != NULL)
	    {
		newRoute = (DSEG)malloc(sizeof(struct dseg_));

		/* The area to paint is derived from the via definitions. */

		if (lefl != NULL)
		{
		    DSEG viaRoute, addRoute;

		    /* If there is a linkedRect structure for the via,	*/
		    /* add those records to the route first.		*/

		    for (viaRoute = lefl->info.via.lr; viaRoute != NULL;
				viaRoute = viaRoute->next)
		    {
			addRoute = (DSEG)malloc(sizeof(struct dseg_));
			addRoute->next = NULL;
			addRoute->layer = viaRoute->layer;

			addRoute->x1 = viaRoute->x1 + refp.x1;
			addRoute->y1 += refp.y1;
			addRoute->x2 += refp.x1;
			addRoute->y2 += refp.y1;

			addRoute->x1 /= 2.0;
			addRoute->y1 /= 2.0;
			addRoute->x2 /= 2.0;
			addRoute->y2 /= 2.0;
	
			if (routeTop)
			    routeList->next = addRoute;
			else
			    routeTop = addRoute;

			routeList = addRoute;
		    }
			
		    paintLayer = lefl->type;

		    newRoute->x1 = refp.x1 + lefl->info.via.area.x1;
		    newRoute->y1 = refp.y1 + lefl->info.via.area.y1;
		    newRoute->x2 = refp.x1 + lefl->info.via.area.x2;
		    newRoute->y2 = refp.y1 + lefl->info.via.area.y2;

		    newRoute->x1 /= 2.0;
		    newRoute->y1 /= 2.0;
		    newRoute->x2 /= 2.0;
		    newRoute->y2 /= 2.0;

		}
		else
		{
		    LefError("Error: Via \"%s\" named but undefined.\n", token);
		    newRoute->x1 = refp.x1 - paintWidth;
		    newRoute->y1 = refp.y1 - paintWidth;
		    newRoute->x2 = refp.x1 + paintWidth;
		    newRoute->y2 = refp.y1 + paintWidth;

		    newRoute->x1 /= 2.0;
		    newRoute->y1 /= 2.0;
		    newRoute->x2 /= 2.0;
		    newRoute->y2 /= 2.0;
		}

		/* After the via, the new route layer becomes whatever	*/
		/* residue of the via was NOT the previous route layer.	*/
		/* This is absolutely impossible to make consistent	*/
		/* with the DEF	spec, but there you have it. . .	*/

		/* to be done. . . */
	    }
	    else
		LefError("Via name \"%s\" unknown in route.\n", token);
	}
	else
	{
	    /* Revert to the routing layer type, in case we painted a via */
	    paintLayer = routeLayer;

	    /* Record current reference point */
	    locarea.x1 = refp.x1;
	    locarea.y1 = refp.y1;

	    /* Read an (X Y) point */
	    token = LefNextToken(f, TRUE);	/* read X */
	    if (*token == '*')
	    {
		if (valid == FALSE)
		{
		    LefError("No reference point for \"*\" wildcard\n"); 
		    goto endCoord;
		}
	    }
	    else if (sscanf(token, "%f", &x) == 1)
	    {
		refp.x1 = (int)roundf((2 * x) / oscale);
	    }
	    else
	    {
		LefError("Cannot parse X coordinate.\n"); 
		goto endCoord;
	    }
	    token = LefNextToken(f, TRUE);	/* read Y */
	    if (*token == '*')
	    {
		if (valid == FALSE)
		{
		    LefError("No reference point for \"*\" wildcard\n"); 
		    free(newRoute);
		    newRoute = NULL;
		    goto endCoord;
		}
	    }
	    else if (sscanf(token, "%f", &y) == 1)
	    {
		refp.y1 = (int)roundf((2 * y) / oscale);
	    }
	    else
	    {
		LefError("Cannot parse Y coordinate.\n"); 
		goto endCoord;
	    }

	    /* Extension is half-width for regular nets, 0 for special nets */
	    /* 0 for special nets is *not* how the 5.3 spec reads, but it   */
	    /* is apparently how everyone interprets it, and is true for    */
	    /* 5.6 spec.						    */

	    z = (special) ? 0 : paintWidth;
	    token = LefNextToken(f, TRUE);
	    if (*token != ')')
	    {
		/* non-default route extension */
		if (sscanf(token, "%f", &z) != 1)
		    LefError("Can't parse route extension value.\n");

		/* all values will be divided by 2, so we need	*/
		/* to multiply up by 2 now.			*/

		else
		    z *= 2;
	    }

	    /* Indicate that we have a valid reference point */

	    if (valid == FALSE)
	    {
		valid = TRUE;
	    }
	    else if ((locarea.x1 != refp.x1) && (locarea.y1 != refp.y1))
	    {
		/* Skip over nonmanhattan segments, reset the reference	*/
		/* point, and output a warning.				*/

		LefError("Can't deal with nonmanhattan geometry in route.\n");
		locarea.x1 = refp.x1;
		locarea.y1 = refp.y1;
	    }
	    else
	    {
		newRoute = (DSEG)malloc(sizeof(struct dseg_));

		/* Route coordinates become the centerline of the	*/
		/* segment.  "refp" is kept in 1/2 lambda units so	*/
		/* we should always end up with integer units.		*/

		locarea.x2 = refp.x1;
		locarea.y2 = refp.y1;

		if (newRoute->x1 == newRoute->x2)
		{
		    newRoute->x1 -= paintWidth;
		    newRoute->x2 += paintWidth;
		}
		else
		{
		    newRoute->x1 -= z;
		    newRoute->x2 += z;
		}

		if (newRoute->y1 == newRoute->y2)
		{
		    newRoute->y1 -= paintWidth;
		    newRoute->y2 += paintWidth;
		}
		else
		{
		    newRoute->y1 -= z;
		    newRoute->y2 += z;
		}

		/* If we don't have integer units here, we should	*/
		/* rescale the magic internal grid.			*/

		newRoute->x1 /= 2.0;
		newRoute->y1 /= 2.0;
		newRoute->x2 /= 2.0;
		newRoute->y2 /= 2.0;
	    }

endCoord:
	    /* Find the closing parenthesis for the coordinate pair */
	    while (*token != ')')
		token = LefNextToken(f, TRUE);
	}

	/* Link in the new route segment */
	if (newRoute)
	{
	    newRoute->layer = paintLayer;
	    newRoute->next = NULL;

	    if (routeTop)
		routeList->next = newRoute;
	    else
		routeTop = newRoute;

	    routeList = newRoute;
	    newRoute = NULL;
	}
    }

    /* Process each segment and paint into the layout */

    while (routeTop != NULL)
    {
	/* to be done:  apply obstructions to grids */

	/* advance to next point and free record */
	routeList = routeTop->next;
	free(routeTop);
	routeTop = routeList;
    }
    return token;	/* Pass back the last token found */
}

/*
 *------------------------------------------------------------
 *
 * DefReadGatePin ---
 *
 *	Given a gate name and a pin name in a net from the
 *	DEF file NETS section, find the position of the
 *	gate, then the position of the pin within the gate,
 *	and add pin and obstruction information to the grid
 *	network.
 *
 *------------------------------------------------------------
 */

DefReadGatePin(NET net, NODE node, char *instname, char *pinname, double *home)
{
    NODE node2;
    int i, j;
    NODELIST nl;
    GATE gateginfo;
    DSEG drect;
    GATE g;
    double dx, dy;
    int gridx, gridy;
    DPOINT dp;

    for (g = Nlgates; g; g = g->next) {
	if (!strcasecmp(instname, g->gatename)) {
	    for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) {
		if (!strcasecmp(g->gatetype, gateginfo->gatetype)) {
		    for (i = 0; i < gateginfo->nodes; i++) {
			if (!strcasecmp(gateginfo->node[i], pinname)) {
			    node->taps = (DPOINT)NULL;
			    node->extend = (DPOINT)NULL;

			    for (drect = g->taps[i]; drect; drect = drect->next) {

				// Add all routing gridpoints that fall inside
				// the rectangle.  Much to do here:
				// (1) routable area should extend 1/2 route width
				// to each side, as spacing to obstructions allows.
				// (2) terminals that are wide enough to route to
				// but not centered on gridpoints should be marked
				// in some way, and handled appropriately.

				gridx = (int)((drect->x1 - Xlowerbound) /
					PitchX[drect->layer]) - 1;
				while (1) {
				    dx = (gridx * PitchX[drect->layer]) + Xlowerbound;
				    if (dx > drect->x2 + home[drect->layer]) break;
				    if (dx < drect->x1 - home[drect->layer]) {
					gridx++;
				 	continue;
				    }
				    gridy = (int)((drect->y1 - Ylowerbound) /
						PitchY[drect->layer]) - 1;
				    while (1) {
					dy = (gridy * PitchY[drect->layer])
						+ Ylowerbound;
					if (dy > drect->y2 + home[drect->layer]) break;
					if (dy < drect->y1 - home[drect->layer]) {
					    gridy++;
					    continue;
					}

					// Routing grid point is an interior point
					// of a gate port.  Record the position

					dp = (DPOINT)malloc(sizeof(struct dpoint_));
					dp->layer = drect->layer;
					dp->x = dx;
					dp->y = dy;
					dp->gridx = gridx;
					dp->gridy = gridy;

					if (dy >= drect->y1 && dx >= drect->x1 &&
						dy <= drect->y2 && dx <= drect->x2) {
					    dp->next = node->taps;
					    node->taps = dp;
					}
					else {
					    dp->next = node->extend;
					    node->extend = dp;
					}
					gridy++;
				    }
				    gridx++;
				}
			    }
			    node->netnum = net->netnum;
			    g->netnum[i] = net->netnum;
			    g->noderec[i] = node;
			    node->netname = net->netname;
			    node->routes = (ROUTE)NULL;
			    node->next = Nlnodes;
			    Nlnodes = node;
			    Numnodes++;
			    break;
			}
		    }
		    if (i < gateginfo->nodes) break;
		}
	    }
	    if (!gateginfo)
		LefError("Endpoint %s/%s of net %s not found\n",
				instname, pinname, net->netname);
	}
    }
}
					   

/*
 *------------------------------------------------------------
 *
 * DefReadNets --
 *
 *	Read a NETS or SPECIALNETS section from a DEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Many.  Networks are created, and geometry may be
 *	painted into the database top-level cell.
 *
 *------------------------------------------------------------
 */

enum def_net_keys {DEF_NET_START = 0, DEF_NET_END};
enum def_netprop_keys {
	DEF_NETPROP_USE = 0, DEF_NETPROP_ROUTED, DEF_NETPROP_FIXED,
	DEF_NETPROP_COVER, DEF_NETPROP_SOURCE, DEF_NETPROP_WEIGHT,
	DEF_NETPROP_PROPERTY};

void
DefReadNets(FILE *f, char *sname, float oscale, char special, int total)
{
    char *token;
    int keyword, subkey;
    int i, processed = 0;
    int nodeidx;
    char netname[MAX_NAME_LEN];
    char instname[MAX_NAME_LEN], pinname[MAX_NAME_LEN];

    NET net;
    NODE node, node2;
    NODELIST nl;
    GATE gateginfo;
    DSEG drect;
    GATE g;
    double dx, dy;
    int gridx, gridy;
    DPOINT dp;
    double home[MAX_LAYERS];

    static char *net_keys[] = {
	"-",
	"END",
	NULL
    };

    static char *net_property_keys[] = {
	"USE",
	"ROUTED",
	"FIXED",
	"COVER",
	"SOURCE",
	"WEIGHT",
	"PROPERTY",
	NULL
    };

    if (special == FALSE)
    {
	// Initialize net and node records
	Nlnets = (NET)NULL;
	Nlnodes = (NODE)NULL;
	Numnets = MIN_NET_NUMBER;

	// Compute distance for keepout halo for each route layer
	for (i = 0; i < Num_layers; i++) {
	    home[i] = LefGetRouteKeepout(i);
	}
    }

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, net_keys);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in NET "
			"definition; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}

	switch (keyword)
	{
	    case DEF_NET_START:

		/* Get net name */
		token = LefNextToken(f, TRUE);

		net = (NET)malloc(sizeof(struct net_));
		net->netnum = Numnets++;
		net->netorder = 0;
		net->netname = strdup(token);
		net->netnodes = (NODELIST)NULL;
		net->noripup = (NETLIST)NULL;

		net->next = Nlnets;
		Nlnets = net;

		nodeidx = 0;

		/* Update the record of the number of nets processed	*/
		/* and spit out a message for every 5% finished.	*/

		processed++;

		/* Get next token;  will be '(' if this is a netlist	*/
		token = LefNextToken(f, TRUE);

		/* Process all properties */
		while (token && (*token != ';'))
		{
		    /* Find connections for the net */
		    if (*token == '(')
		    {
			token = LefNextToken(f, TRUE);  /* get pin or gate */
			strcpy(instname, token);
			token = LefNextToken(f, TRUE);	/* get node name */

			if (!strcasecmp(instname, "pin")) {
			    strcpy(instname, token);
			    strcpy(pinname, "pin");
			}
			else
			    strcpy(pinname, token);

			node = (NODE)calloc(1, sizeof(struct node_));
			node->nodenum = nodeidx++;
			DefReadGatePin(net, node, instname, pinname, home);

			token = LefNextToken(f, TRUE);	/* should be ')' */

			continue;
		    }
		    else if (*token != '+')
		    {
			token = LefNextToken(f, TRUE);	/* Not a property */
			continue;	/* Ignore it, whatever it is */
		    }
		    else
			token = LefNextToken(f, TRUE);

		    subkey = Lookup(token, net_property_keys);
		    if (subkey < 0)
		    {
			LefError("Unknown net property \"%s\" in "
				"NET definition; ignoring.\n", token);
			continue;
		    }
		    switch (subkey)
		    {
			case DEF_NETPROP_USE:
			    /* Presently, we ignore this */
			    break;
			case DEF_NETPROP_ROUTED:
			case DEF_NETPROP_FIXED:
			case DEF_NETPROP_COVER:
			    // For now, absorb an already placed route
			    while (token && (*token != ';'))
			        token = DefAddRoutes(f, oscale, special);
			    break;
		    }
		}
		break;

	    case DEF_NET_END:
		if (!LefParseEndStatement(f, sname))
		{
		    LefError("Net END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == DEF_NET_END) break;
    }

    // Set the number of nodes per net for each node on the net

    if (special == FALSE) {
	for (node = Nlnodes; node; node = node->next) {
	    i = 0;
	    for (node2 = Nlnodes; node2; node2 = node2->next) {
		if (node->netname == node2->netname) i++;
	    }
	    node->numnodes = i;
	}

	// Fill in the netnodes list for each net, needed for checking
	// for isolated routed groups within a net.

	for (net = Nlnets; net; net = net->next) {
	    for (node = Nlnodes; node; node = node->next) {
		if (node->netname == net->netname) {
		    nl = (NODELIST)malloc(sizeof(struct nodelist_));
		    nl->node = node;
		    nl->next = net->netnodes;
		    net->netnodes = nl;
		    net->numnodes++;
		}
	    }
	}
    }

    if (processed == total)
	fprintf(stdout, "  Processed %d%s nets total.\n", processed,
		(special) ? " special" : "");
    else
	LefError("Warning:  Number of nets read (%d) does not match "
		"the number declared (%d).\n", processed, total);
}

/*
 *------------------------------------------------------------
 *
 * DefReadUseLocation --
 *
 *	Read location and orientation of a cell use
 *	Syntax: ( X Y ) O
 *
 * Results:
 *	0 on success, -1 on failure
 *
 * Side Effects:
 *	GATE definition for the use has the placedX, placedY,
 *	and orient values filled.
 *------------------------------------------------------------
 */
enum def_orient {DEF_NORTH, DEF_SOUTH, DEF_EAST, DEF_WEST,
	DEF_FLIPPED_NORTH, DEF_FLIPPED_SOUTH, DEF_FLIPPED_EAST,
	DEF_FLIPPED_WEST};

int
DefReadLocation(gate, f, oscale)
    GATE gate;
    FILE *f;
    float oscale;
{
    DSEG r;
    struct dseg_ tr;
    int keyword;
    char *token;
    float x, y;
    char mxflag, myflag;

    static char *orientations[] = {
	"N", "S", "E", "W", "FN", "FS", "FE", "FW"
    };

    token = LefNextToken(f, TRUE);
    if (*token != '(') goto parse_error;
    token = LefNextToken(f, TRUE);
    if (sscanf(token, "%f", &x) != 1) goto parse_error;
    token = LefNextToken(f, TRUE);
    if (sscanf(token, "%f", &y) != 1) goto parse_error;
    token = LefNextToken(f, TRUE);
    if (*token != ')') goto parse_error;
    token = LefNextToken(f, TRUE);

    keyword = Lookup(token, orientations);
    if (keyword < 0)
    {
	LefError("Unknown macro orientation \"%s\".\n", token);
	return -1;
    }

    mxflag = myflag = (char)0;

    switch (keyword)
    {
	case DEF_NORTH:
	    break;
	case DEF_SOUTH:
	    mxflag = 1;
	    myflag = 1;
	    break;
	case DEF_FLIPPED_NORTH:
	    mxflag = 1;
	    break;
	case DEF_FLIPPED_SOUTH:
	    myflag = 1;
	    break;
	case DEF_EAST:
	case DEF_WEST:
	case DEF_FLIPPED_EAST:
	case DEF_FLIPPED_WEST:
	    LefError("Error:  Cannot handle 90-degree rotated components!\n");
	    break;
    }

    if (gate) {
	gate->placedX = x / oscale;
	gate->placedY = y / oscale;
	gate->orient = MNONE;
	if (mxflag) gate->orient |= MX;
	if (myflag) gate->orient |= MY;
    }
    return 0;

parse_error:
    LefError("Cannot parse location: must be ( X Y ) orient\n");
    return -1;
}

/*
 *------------------------------------------------------------
 *
 * DefReadPins --
 *
 *	Read a PINS section from a DEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Generates paint and labels in the layout.
 *
 *------------------------------------------------------------
 */

enum def_pins_keys {DEF_PINS_START = 0, DEF_PINS_END};
enum def_pins_prop_keys {
	DEF_PINS_PROP_NET = 0, DEF_PINS_PROP_DIR,
	DEF_PINS_PROP_LAYER, DEF_PINS_PROP_PLACED,
	DEF_PINS_PROP_USE, DEF_PINS_PROP_FIXED,
	DEF_PINS_PROP_COVER};

void
DefReadPins(FILE *f, char *sname, float oscale, int total)
{
    char *token;
    char pinname[LEF_LINE_MAX];
    int keyword, subkey, values;
    int processed = 0;
    int pinDir = PORT_CLASS_DEFAULT;
    DSEG currect, drect;
    GATE gate;
    int curlayer;
    double hwidth;

    static char *pin_keys[] = {
	"-",
	"END",
	NULL
    };

    static char *pin_property_keys[] = {
	"NET",
	"DIRECTION",
	"LAYER",
	"PLACED",
	"USE",
	"FIXED",
	"COVER",
	NULL
    };

    static char *pin_classes[] = {
	"DEFAULT",
	"INPUT",
	"OUTPUT TRISTATE",
	"OUTPUT",
	"INOUT",
	"FEEDTHRU",
	NULL
    };

    static int lef_class_to_bitmask[] = {
	PORT_CLASS_DEFAULT,
	PORT_CLASS_INPUT,
	PORT_CLASS_TRISTATE,
	PORT_CLASS_OUTPUT,
	PORT_CLASS_BIDIRECTIONAL,
	PORT_CLASS_FEEDTHROUGH
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, pin_keys);

	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in PINS "
			"definition; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case DEF_PINS_START:		/* "-" keyword */

		/* Update the record of the number of pins		*/
		/* processed and spit out a message for every 5% done.	*/
 
		processed++;

		/* Get pin name */
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%2047s", pinname) != 1)
		{
		    LefError("Bad pin statement:  Need pin name\n");
		    LefEndStatement(f);
		    break;
		}

		/* Create the pin record */
		gate = (GATE)malloc(sizeof(struct gate_));
		gate->gatetype = (char *)malloc(4);
		strcpy(gate->gatetype, "pin");
		Numpins++;
		gate->gatename = NULL;	/* Use NET, but if none, use	*/
					/* the pin name, set at end.	*/
		gate->width = gate->height = 0;
		curlayer = -1;

		/* Now do a search through the line for "+" entries	*/
		/* And process each.					*/

		while ((token = LefNextToken(f, TRUE)) != NULL)
		{
		    if (*token == ';') break;
		    if (*token != '+') continue;

		    token = LefNextToken(f, TRUE);
		    subkey = Lookup(token, pin_property_keys);
		    if (subkey < 0)
		    {
			LefError("Unknown pin property \"%s\" in "
				"PINS definition; ignoring.\n", token);
			continue;
		    }
		    switch (subkey)
		    {
			case DEF_PINS_PROP_NET:
			    /* Get the net name */
			    token = LefNextToken(f, TRUE);
			    gate->gatename = strdup(token);
			    gate->node[0] = strdup(token);
			    break;
			case DEF_PINS_PROP_DIR:
			    token = LefNextToken(f, TRUE);
			    subkey = Lookup(token, pin_classes);
			    if (subkey < 0)
				LefError("Unknown pin class\n");
			    else
				pinDir = lef_class_to_bitmask[subkey];
			    break;
			case DEF_PINS_PROP_LAYER:
			    curlayer = LefReadLayer(f, FALSE);
			    currect = LefReadRect(f, curlayer, oscale);
			    gate->width = currect->x2 - currect->x1;
			    gate->height = currect->y2 - currect->y1;
			    break;
			case DEF_PINS_PROP_USE:
			    /* Ignore this, for now */
			    break;
			case DEF_PINS_PROP_PLACED:
			case DEF_PINS_PROP_FIXED:
			case DEF_PINS_PROP_COVER:
			    DefReadLocation(gate, f, oscale);
			    break;
		    }
		}

		if (curlayer >= 0 && curlayer < Num_layers) {

		    /* If no NET was declared for pin, use pinname */
		    if (gate->gatename == NULL)
			gate->gatename = strdup(pinname);

		    /* Make sure pin is at least the size of the route layer */
		    drect = (DSEG)malloc(sizeof(struct dseg_));
		    gate->taps[0] = drect;
		    drect->next = (DSEG)NULL;

		    hwidth = LefGetRouteWidth(curlayer);
		    if (gate->width < hwidth) gate->width = hwidth;
		    if (gate->height < hwidth) gate->height = hwidth;
		    hwidth /= 2.0;
		    drect->x1 = gate->placedX - hwidth;
		    drect->y1 = gate->placedY - hwidth;
		    drect->x2 = gate->placedX + hwidth;
		    drect->y2 = gate->placedY + hwidth;
		    drect->layer = curlayer;
		    gate->obs = (DSEG)NULL;
		    gate->nodes = 1;
		    gate->glue = TRUE;
		    gate->vert = FALSE;
		    gate->next = Nlgates;
		    Nlgates = gate;
		}
		else {
		    LefError("Pin %s is defined outside of route layer area!\n",
				pinname);
		    free(gate);
		}

		break;

	    case DEF_PINS_END:
		if (!LefParseEndStatement(f, sname))
		{
		    LefError("Pins END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == DEF_PINS_END) break;
    }

    if (processed == total)
	fprintf(stdout, "  Processed %d pins total.\n", processed);
    else
	LefError("Warning:  Number of pins read (%d) does not match "
		"the number declared (%d).\n", processed, total);
}
 
/*
 *------------------------------------------------------------
 *
 * DefReadVias --
 *
 *	Read a VIAS section from a DEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Technically, this routine should be creating a cell for
 *	each defined via.  For now, it just computes the bounding
 *	rectangle and layer.
 *
 *------------------------------------------------------------
 */

enum def_vias_keys {DEF_VIAS_START = 0, DEF_VIAS_END};
enum def_vias_prop_keys {
	DEF_VIAS_PROP_RECT = 0};

void
DefReadVias(f, sname, oscale, total)
    FILE *f;
    char *sname;
    float oscale;
    int total;
{
    char *token;
    char vianame[LEF_LINE_MAX];
    int keyword, subkey, values;
    int processed = 0;
    int curlayer;
    DSEG currect;
    LefList lefl;

    static char *via_keys[] = {
	"-",
	"END",
	NULL
    };

    static char *via_property_keys[] = {
	"RECT",
	NULL
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, via_keys);

	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in VIAS "
			"definition; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case DEF_VIAS_START:		/* "-" keyword */

		/* Update the record of the number of vias		*/
		/* processed and spit out a message for every 5% done.	*/
 
		processed++;

		/* Get via name */
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%2047s", vianame) != 1)
		{
		    LefError("Bad via statement:  Need via name\n");
		    LefEndStatement(f);
		    break;
		}
		lefl = LefFindLayer(token);
                if (lefl == NULL)
                {
                    lefl = (LefList)calloc(1, sizeof(lefLayer));
                    lefl->type = -1;
                    lefl->obsType = -1;
                    lefl->lefClass = CLASS_VIA;
                    lefl->info.via.area.x1 = 0.0;
                    lefl->info.via.area.y1 = 0.0;
                    lefl->info.via.area.x2 = 0.0;
                    lefl->info.via.area.y2 = 0.0;
                    lefl->info.via.area.layer = -1;
                    lefl->info.via.cell = (GATE)NULL;
                    lefl->info.via.lr = (DSEG)NULL;
                    lefl->lefName = strdup(token);

                    lefl->next = LefInfo;
                    LefInfo = lefl;
		}
		else
		{
		    LefError("Warning:  Composite via \"%s\" redefined.\n", vianame);
		    lefl = LefRedefined(lefl, vianame);
		}

		/* Now do a search through the line for "+" entries	*/
		/* And process each.					*/

		while ((token = LefNextToken(f, TRUE)) != NULL)
		{
		    if (*token == ';') break;
		    if (*token != '+') continue;

		    token = LefNextToken(f, TRUE);
		    subkey = Lookup(token, via_property_keys);
		    if (subkey < 0)
		    {
			LefError("Unknown via property \"%s\" in "
				"VIAS definition; ignoring.\n", token);
			continue;
		    }
		    switch (subkey)
		    {
			case DEF_VIAS_PROP_RECT:
			    curlayer = LefReadLayer(f, FALSE);
			    LefAddViaGeometry(f, lefl, curlayer, oscale);
			    break;
		    }
		}
		break;

	    case DEF_VIAS_END:
		if (!LefParseEndStatement(f, sname))
		{
		    LefError("Vias END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == DEF_VIAS_END) break;
    }

    if (processed == total)
	fprintf(stdout, "  Processed %d vias total.\n", processed);
    else
	LefError("Warning:  Number of vias read (%d) does not match "
		"the number declared (%d).\n", processed, total);
}
 
/*
 *------------------------------------------------------------
 *
 * DefReadComponents --
 *
 *	Read a COMPONENTS section from a DEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Many.  Cell instances are created and added to
 *	the database.
 *
 *------------------------------------------------------------
 */

enum def_comp_keys {DEF_COMP_START = 0, DEF_COMP_END};
enum def_prop_keys {
	DEF_PROP_FIXED = 0, DEF_PROP_COVER,
	DEF_PROP_PLACED, DEF_PROP_UNPLACED,
	DEF_PROP_SOURCE, DEF_PROP_WEIGHT, DEF_PROP_FOREIGN,
	DEF_PROP_REGION, DEF_PROP_GENERATE, DEF_PROP_PROPERTY,
	DEF_PROP_EEQMASTER};

void
DefReadComponents(FILE *f, char *sname, float oscale, int total)
{
    GATE gateginfo;
    GATE gate;
    char *token;
    char usename[512];
    int keyword, subkey, values, i;
    int processed = 0;
    char OK;
    DSEG drect, newrect;
    double tmp, maxx, minx, maxy, miny;

    static char *component_keys[] = {
	"-",
	"END",
	NULL
    };

    static char *property_keys[] = {
	"FIXED",
	"COVER",
	"PLACED",
	"UNPLACED",
	"SOURCE",
	"WEIGHT",
	"FOREIGN",
	"REGION",
	"GENERATE",
	"PROPERTY",
	"EEQMASTER",
	NULL
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, component_keys);

	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in COMPONENT "
			"definition; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case DEF_COMP_START:		/* "-" keyword */

		/* Update the record of the number of components	*/
		/* processed and spit out a message for every 5% done.	*/
 
		processed++;

		/* Get use and macro names */
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%511s", usename) != 1)
		{
		    LefError("Bad component statement:  Need use and macro names\n");
		    LefEndStatement(f);
		    break;
		}
		token = LefNextToken(f, TRUE);

		/* Find the corresponding macro */
		OK = 0;
		for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) {
		    if (!strcasecmp(gateginfo->gatetype, token)) {
			OK = 1;
			break;
		    }
		}
		if (!OK) {
		    LefError("Could not find a macro definition for \"%s\"\n",
				token);
		    gate = NULL;
		}
		else {
		    gate = (GATE)malloc(sizeof(struct gate_));
		    gate->gatename = strdup(usename);
		    gate->gatetype = strdup(token);
		    gate->gatenum = processed;
		}
		if (Numgates < processed) Numgates = processed;

		
		/* Now do a search through the line for "+" entries	*/
		/* And process each.					*/

		while ((token = LefNextToken(f, TRUE)) != NULL)
		{
		    if (*token == ';') break;
		    if (*token != '+') continue;

		    token = LefNextToken(f, TRUE);
		    subkey = Lookup(token, property_keys);
		    if (subkey < 0)
		    {
			LefError("Unknown component property \"%s\" in "
				"COMPONENT definition; ignoring.\n", token);
			continue;
		    }
		    switch (subkey)
		    {
			case DEF_PROP_PLACED:
			case DEF_PROP_UNPLACED:
			case DEF_PROP_FIXED:
			case DEF_PROP_COVER:
			    DefReadLocation(gate, f, oscale);
			    break;
			case DEF_PROP_SOURCE:
			case DEF_PROP_WEIGHT:
			case DEF_PROP_FOREIGN:
			case DEF_PROP_REGION:
			case DEF_PROP_GENERATE:
			case DEF_PROP_PROPERTY:
			case DEF_PROP_EEQMASTER:
			    token = LefNextToken(f, TRUE);
			    break;
		    }
		}

		if (gate != NULL)
		{
		    /* Process the gate */
		    gate->width = gateginfo->width;   
		    gate->height = gateginfo->height;   
		    gate->nodes = gateginfo->nodes;   
		    gate->obs = (DSEG)NULL;

		    for (i = 0; i < gate->nodes; i++) {
			/* Let the node names point to the master cell;	*/
			/* this is just diagnostic;  allows us, for	*/
			/* instance, to identify vdd and gnd nodes, so	*/
			/* we don't complain about them being		*/
			/* disconnected.				*/

			gate->node[i] = gateginfo->node[i];  /* copy pointer */
			gate->taps[i] = (DSEG)NULL;

			/* Make a copy of the gate nodes and adjust for	*/
			/* instance position				*/

			for (drect = gateginfo->taps[i]; drect; drect = drect->next) {
			    newrect = (DSEG)malloc(sizeof(struct dseg_));
			    *newrect = *drect;
			    newrect->next = gate->taps[i];
			    gate->taps[i] = newrect;
			}

			for (drect = gate->taps[i]; drect; drect = drect->next) {
			    // handle offset from gate origin
			    drect->x1 -= gateginfo->placedX;
			    drect->x2 -= gateginfo->placedX;
			    drect->y1 -= gateginfo->placedY;
			    drect->y2 -= gateginfo->placedY;

			    // handle rotations and orientations here
			    if (gate->orient & MX) {
				tmp = drect->x1;
				drect->x1 = -drect->x2;
				drect->x1 += gate->placedX + gateginfo->width;
				drect->x2 = -tmp;
				drect->x2 += gate->placedX + gateginfo->width;
			    }
			    else {
				drect->x1 += gate->placedX;
				drect->x2 += gate->placedX;
			    }
			    if (gate->orient & MY) {
				tmp = drect->y1;
				drect->y1 = -drect->y2;
				drect->y1 += gate->placedY + gateginfo->height;
				drect->y2 = -tmp;
				drect->y2 += gate->placedY + gateginfo->height;
			    }
			    else {
				drect->y1 += gate->placedY;
				drect->y2 += gate->placedY;
			    }
		
			    if (drect->x1 > maxx) maxx = drect->x1;
			    else if (drect->x1 < minx) minx = drect->x1;
			    if (drect->x2 > maxx) maxx = drect->x2;
			    else if (drect->x2 < minx) minx = drect->x2;

			    if (drect->y1 > maxy) maxy = drect->y1;
			    else if (drect->y1 < miny) miny = drect->y1;
			    if (drect->y2 > maxy) maxy = drect->y2;
			    else if (drect->y2 < miny) miny = drect->y2;
			}
		    }

		    /* Make a copy of the gate obstructions and adjust	*/
		    /* for instance position				*/
		    for (drect = gateginfo->obs; drect; drect = drect->next) {
			newrect = (DSEG)malloc(sizeof(struct dseg_));
			*newrect = *drect;
			newrect->next = gate->obs;
			gate->obs = newrect;
		    }

		    for (drect = gate->obs; drect; drect = drect->next) {
			drect->x1 -= gateginfo->placedX;
			drect->x2 -= gateginfo->placedX;
			drect->y1 -= gateginfo->placedY;
			drect->y2 -= gateginfo->placedY;

			// handle rotations and orientations here
			if (gate->orient & MX) {
			    tmp = drect->x1;
			    drect->x1 = -drect->x2;
			    drect->x1 += gate->placedX + gateginfo->width;
			    drect->x2 = -tmp;
			    drect->x2 += gate->placedX + gateginfo->width;
			}
			else {
			    drect->x1 += gate->placedX;
			    drect->x2 += gate->placedX;
			}
			if (gate->orient & MY) {
			    tmp = drect->y1;
			    drect->y1 = -drect->y2;
			    drect->y1 += gate->placedY + gateginfo->height;
			    drect->y2 = -tmp;
			    drect->y2 += gate->placedY + gateginfo->height;
			}
			else {
			    drect->y1 += gate->placedY;
			    drect->y2 += gate->placedY;
			}
		    }
		    gate->next = Nlgates;
		    Nlgates = gate;
		}
		break;

	    case DEF_COMP_END:
		if (!LefParseEndStatement(f, sname))
		{
		    LefError("Component END statement missing.\n");
		    keyword = -1;
		}

		/* Finish final call by placing the cell use */
		if ((total > 0) && (gate != NULL))
		{
		    // Nothing to do. . . gate has already been placed in list.
		    gate = NULL;
		}
		break;
	}
	if (keyword == DEF_COMP_END) break;
    }

    if (processed == total)
	fprintf(stdout, "  Processed %d subcell instances total.\n", processed);
    else
	LefError("Warning:  Number of subcells read (%d) does not match "
		"the number declared (%d).\n", processed, total);
}

/*
 *------------------------------------------------------------
 *
 * DefRead --
 *
 *	Read a .def file and parse die area, track positions,
 *	components, pins, and nets.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Many.
 *
 *------------------------------------------------------------
 */

/* Enumeration of sections defined in DEF files */

enum def_sections {DEF_VERSION = 0, DEF_NAMESCASESENSITIVE,
	DEF_UNITS, DEF_DESIGN, DEF_REGIONS, DEF_ROW, DEF_TRACKS,
	DEF_GCELLGRID, DEF_DIVIDERCHAR, DEF_BUSBITCHARS,
	DEF_PROPERTYDEFINITIONS, DEF_DEFAULTCAP, DEF_TECHNOLOGY,
	DEF_HISTORY, DEF_DIEAREA, DEF_COMPONENTS, DEF_VIAS,
	DEF_PINS, DEF_PINPROPERTIES, DEF_SPECIALNETS,
	DEF_NETS, DEF_IOTIMINGS, DEF_SCANCHAINS,
	DEF_CONSTRAINTS, DEF_GROUPS, DEF_EXTENSION,
	DEF_END};

void
DefRead(char *inName)
{
    FILE *f;
    char filename[256];
    char *token;
    int keyword, dscale, total;
    int curlayer, channels;
    int v, h, i;
    float oscale;
    double start, step;
    double llx, lly, urx, ury;
    char corient = '.';
    DSEG diearea;

    static char *sections[] = {
	"VERSION",
	"NAMESCASESENSITIVE",
	"UNITS",
	"DESIGN",
	"REGIONS",
	"ROW",
	"TRACKS",
	"GCELLGRID",
	"DIVIDERCHAR",
	"BUSBITCHARS",
	"PROPERTYDEFINITIONS",
	"DEFAULTCAP",
	"TECHNOLOGY",
	"HISTORY",
	"DIEAREA",
	"COMPONENTS",
	"VIAS",
	"PINS",
	"PINPROPERTIES",
	"SPECIALNETS",
	"NETS",
	"IOTIMINGS",
	"SCANCHAINS",
	"CONSTRAINTS",
	"GROUPS",
	"BEGINEXT",
	"END",
	NULL
    };

    if (!strrchr(inName, '.'))
	sprintf(filename, "%s.def", inName);
    else
	strcpy(filename, inName);
   
    f = fopen(filename, "r");

    if (f == NULL)
    {
	fprintf(stderr, "Cannot open input file: ");
	perror(filename);
	return;
    }

    /* Initialize */

    fprintf(stdout, "Reading DEF data from file %s.\n", filename);
    fflush(stdout);

    oscale = 1;
    lefCurrentLine = 0;
    v = h = -1;

    /* Read file contents */

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, sections);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in DEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}

	/* After the TRACKS have been read in, corient is 'x' or 'y'.	*/
	/* On the next keyword, finish filling in track information.	*/

	if (keyword != DEF_TRACKS && corient != '.')
	{
	    /* Because the TRACKS statement only covers the pitch of	*/
	    /* a single direction, we need to fill in with the pitch	*/
	    /* of opposing layers.  For now, we expect all horizontal	*/
	    /* routes to be at the same pitch, and all vertical routes	*/
	    /* to be at the same pitch.					*/

	    if (h == -1) h = v;
	    if (v == -1) v = h;

	    /* This code copied from config.c.  Preferably, all		*/
	    /* information available in the DEF file should be taken	*/
	    /* from the DEF file.					*/

	    for (i = 0; i < Num_layers; i++)
	    {
		if (PitchX[i] != 0.0 && PitchX[i] != PitchX[v])
		    fprintf(stderr, "Multiple vertical route layers at different"
				" pitches.  Using pitch %g and routing on 1-of-N"
				" tracks for larger pitches.\n",
				PitchX[v]);
		PitchX[i] = PitchX[v];
		if (PitchY[i] != 0.0 && PitchY[i] != PitchY[h])
		    fprintf(stderr, "Multiple horizontal route layers at different"
				" pitches.  Using pitch %g and routing on 1-of-N"
				" tracks for larger pitches.\n",
				PitchY[h]);
		PitchY[i] = PitchY[h];

		corient = '.';	// So we don't run this code again.
	    }
	}

	switch (keyword)
	{
	    case DEF_VERSION:
		LefEndStatement(f);
		break;
	    case DEF_NAMESCASESENSITIVE:
		LefEndStatement(f);
		break;
	    case DEF_TECHNOLOGY:
		token = LefNextToken(f, TRUE);
		fprintf(stdout, "Diagnostic: DEF file technology: \"%s\"\n", token);
		LefEndStatement(f);
	 	break;
	    case DEF_REGIONS:
		LefSkipSection(f, sections[DEF_REGIONS]);
		break;
	    case DEF_DESIGN:
		token = LefNextToken(f, TRUE);
		fprintf(stdout, "Diagnostic: Design name: \"%s\"\n", token);
		LefEndStatement(f);
		break;
	    case DEF_UNITS:
		token = LefNextToken(f, TRUE);
		token = LefNextToken(f, TRUE);
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &dscale) != 1)
		{
		    LefError("Invalid syntax for UNITS statement.\n");
		    LefError("Assuming default value of 100\n");
		    dscale = 100;
		}
		/* We don't care if the scale is 100, 200, 1000, or 2000. */
		/* Do we need to deal with numeric roundoff issues?	  */
		oscale *= (float)dscale;
		LefEndStatement(f);
		break;
	    case DEF_ROW:
		LefEndStatement(f);
		break;
	    case DEF_TRACKS:
		token = LefNextToken(f, TRUE);
		if (strlen(token) != 1) {
		    LefError("Problem parsing track orientation (X or Y).\n");
		}
		corient = tolower(token[0]);	// X or Y
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%lg", &start) != 1) {
		    LefError("Problem parsing track start position.\n");
		}
		token = LefNextToken(f, TRUE);
		if (strcmp(token, "DO")) {
		    LefError("TRACKS missing DO loop.\n");
		}
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &channels) != 1) {
		    LefError("Problem parsing number of track channels.\n");
		}
		token = LefNextToken(f, TRUE);
		if (strcmp(token, "STEP")) {
		    LefError("TRACKS missing STEP size.\n");
		}
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%lg", &step) != 1) {
		    LefError("Problem parsing track step size.\n");
		}
		token = LefNextToken(f, TRUE);
		if (!strcmp(token, "LAYER")) {
		    curlayer = LefReadLayer(f, FALSE);
		}
		if (corient == 'x') {
		    Vert[curlayer] = 1;
		    PitchX[curlayer] = step / oscale;
		    if ((v == -1) || (PitchX[curlayer] < PitchX[v])) v = curlayer;
		    if ((curlayer < Num_layers - 1) && PitchX[curlayer + 1] == 0.0)
			PitchX[curlayer + 1] = PitchX[curlayer];
		    llx = start;
		    urx = start + step * channels;
		    if (llx < Xlowerbound) Xlowerbound = llx / oscale;
		    if (urx > Xupperbound) Xupperbound = urx / oscale;
		}
		else {
		    Vert[curlayer] = 0;
		    PitchY[curlayer] = step / oscale;
		    if ((h == -1) || (PitchY[curlayer] < PitchX[h])) h = curlayer;
		    if ((curlayer < Num_layers - 1) && PitchY[curlayer + 1] == 0.0)
			PitchY[curlayer + 1] = PitchY[curlayer];
		    lly = start;
		    ury = start + step * channels;
		    if (lly < Ylowerbound) Ylowerbound = lly / oscale;
		    if (ury > Yupperbound) Yupperbound = ury / oscale;
		}
		LefEndStatement(f);
		break;
	    case DEF_GCELLGRID:
		LefEndStatement(f);
		break;
	    case DEF_DIVIDERCHAR:
		LefEndStatement(f);
		break;
	    case DEF_BUSBITCHARS:
		LefEndStatement(f);
		break;
	    case DEF_HISTORY:
		LefEndStatement(f);
		break;
	    case DEF_DIEAREA:
		diearea = LefReadRect(f, 0, oscale); // no current layer, use 0
		Xlowerbound = diearea->x1;
		Ylowerbound = diearea->y1;
		Xupperbound = diearea->x2;
		Yupperbound = diearea->y2;
		LefEndStatement(f);
		break;
	    case DEF_PROPERTYDEFINITIONS:
		LefSkipSection(f, sections[DEF_PROPERTYDEFINITIONS]);
		break;
	    case DEF_DEFAULTCAP:
		LefSkipSection(f, sections[DEF_DEFAULTCAP]);
		break;
	    case DEF_COMPONENTS:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &total) != 1) total = 0;
		LefEndStatement(f);
		DefReadComponents(f, sections[DEF_COMPONENTS], oscale, total);
		break;
	    case DEF_VIAS:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &total) != 1) total = 0;
		LefEndStatement(f);
		DefReadVias(f, sections[DEF_VIAS], oscale, total);
		break;
	    case DEF_PINS:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &total) != 1) total = 0;
		LefEndStatement(f);
		DefReadPins(f, sections[DEF_PINS], oscale, total);
		break;
	    case DEF_PINPROPERTIES:
		LefSkipSection(f, sections[DEF_PINPROPERTIES]);
		break;
	    case DEF_SPECIALNETS:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &total) != 1) total = 0;
		LefEndStatement(f);
		DefReadNets(f, sections[DEF_SPECIALNETS], oscale, TRUE, total);
		break;
	    case DEF_NETS:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%d", &total) != 1) total = 0;
		LefEndStatement(f);
		DefReadNets(f, sections[DEF_NETS], oscale, FALSE, total);
		break;
	    case DEF_IOTIMINGS:
		LefSkipSection(f, sections[DEF_IOTIMINGS]);
		break;
	    case DEF_SCANCHAINS:
		LefSkipSection(f, sections[DEF_SCANCHAINS]);
		break;
	    case DEF_CONSTRAINTS:
		LefSkipSection(f, sections[DEF_CONSTRAINTS]);
		break;
	    case DEF_GROUPS:
		LefSkipSection(f, sections[DEF_GROUPS]);
		break;
	    case DEF_EXTENSION:
		LefSkipSection(f, sections[DEF_EXTENSION]);
		break;
	    case DEF_END:
		if (!LefParseEndStatement(f, "DESIGN"))
		{
		    LefError("END statement out of context.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == DEF_END) break;
    }
    fprintf(stdout, "DEF read: Processed %d lines.\n", lefCurrentLine);
    LefError(NULL);	/* print statement of errors, if any, and reset */

    /* Cleanup */

    if (f != NULL) fclose(f);
}

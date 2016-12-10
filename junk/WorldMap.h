/*
 * APRSWorldMap.h
 *
 *  Created on: Jun 16, 2015
 *      Author: dongfang
 */

#ifndef INC_WORLDMAP_H_
#define INC_WORLDMAP_H_

// End of polygon list.
#define POLYGON_LIST_END 0xffff

// Fit a (lat,lon) into a single uint16. There are 181 different latitudes...
#define encodeLonLat(lon,lat) ((uint16_t)(((lon)+180)*181 + (lat)+90))

#define BOUNDARY_144800 {\
 /* Africa and Europe */\
 encodeLonLat(19,-38),\
 encodeLonLat(-19,14),\
 encodeLonLat(-19,32),\
 encodeLonLat(-10,57),\
 encodeLonLat(20,72),\
 encodeLonLat(75,70),\
 encodeLonLat(75,39),\
 encodeLonLat(29,-36),\
 encodeLonLat(19,-38),\
 \
 /* Russia (too large? Hopeless? */\
 encodeLonLat(75,70),\
 encodeLonLat(180,70),\
 encodeLonLat(180,60),\
 encodeLonLat(75,39),\
 encodeLonLat(75,70),\
 \
 /* Costa Rica */\
 encodeLonLat(-87,11),\
 encodeLonLat(-83,11),\
 encodeLonLat(-82,10),\
 encodeLonLat(-84,7),\
 encodeLonLat(-87,11),\
 \
 /* Cape Verde */\
 encodeLonLat(-26,14),\
 encodeLonLat(-26,18),\
 encodeLonLat(-22,17),\
 encodeLonLat(-22,14),\
 encodeLonLat(-26,14),\
 \
 /* Azores */\
 encodeLonLat(-32,41),\
 encodeLonLat(-24,38),\
 encodeLonLat(-24,36),\
 encodeLonLat(-32,39),\
 encodeLonLat(-32,41),\
 \
 POLYGON_LIST_END \
};

#define BOUNDARY_144640 {\
 /* China */\
 encodeLonLat(86,49),\
 encodeLonLat(135,49),\
 encodeLonLat(122,21),\
 encodeLonLat(107,17),\
 encodeLonLat(75,39),\
 encodeLonLat(86,49),\
 \
 POLYGON_LIST_END \
};

#define BOUNDARY_144660 {\
 /* Japan */\
 encodeLonLat(128,33),\
 encodeLonLat(141,46),\
 encodeLonLat(147,44),\
 encodeLonLat(142,34),\
 encodeLonLat(131,30),\
 encodeLonLat(129,30),\
 encodeLonLat(128,33),\
 \
 /* Japanese islands */\
 encodeLonLat(123,23),\
 encodeLonLat(123,26),\
 encodeLonLat(129,30),\
 encodeLonLat(131,30),\
 encodeLonLat(133,25),\
 encodeLonLat(123,23),\
 POLYGON_LIST_END \
};

#define BOUNDARY_144620 {\
 /* S Korea */\
 encodeLonLat(128,33),\
 encodeLonLat(124,33),\
 encodeLonLat(125,38),\
 encodeLonLat(130,40),\
 encodeLonLat(132,37),\
 encodeLonLat(128,33),\
 \
 POLYGON_LIST_END \
};

#define BOUNDARY_144390 {\
 /* Continental US */\
 encodeLonLat(-94,12),\
 encodeLonLat(-112,18),\
 encodeLonLat(-170,60),\
 encodeLonLat(-170,70),\
 encodeLonLat(-90,70),\
 encodeLonLat(-54,53),\
 encodeLonLat(-54,46),\
 encodeLonLat(-78,25),\
 encodeLonLat(-94,12),\
 \
 /* Hawaii */\
 encodeLonLat(-161,21),\
 encodeLonLat(-160,24),\
 encodeLonLat(-154,20),\
 encodeLonLat(-155,18),\
 encodeLonLat(-161,21),\
 \
 /* Midway Is */\
 encodeLonLat(-179,29),\
 encodeLonLat(-176,29),\
 encodeLonLat(-176,27),\
 encodeLonLat(-179,27),\
 encodeLonLat(-179,29),\
 \
 /* Guam */\
 encodeLonLat(144,12),\
 encodeLonLat(144,20),\
 encodeLonLat(147,20),\
 encodeLonLat(147,15),\
 encodeLonLat(145,12),\
 encodeLonLat(144,12),\
 \
 /* Chile 1 */\
 encodeLonLat(-71,-17),\
 encodeLonLat(-66,-17),\
 encodeLonLat(-73,-51),\
 encodeLonLat(-78,-50),\
 encodeLonLat(-71,-17),\
 \
 /* Chile 2 */\
 encodeLonLat(-73,-51),\
 encodeLonLat(-64,-55),\
 encodeLonLat(-70,-57),\
 encodeLonLat(-78,-50),\
 encodeLonLat(-73,-51),\
 \
 /* Indonesia */\
 encodeLonLat(133,-10),\
 encodeLonLat(104,-10),\
 encodeLonLat(93,6),\
 encodeLonLat(118,6),\
 encodeLonLat(143,-2),\
 encodeLonLat(143,-10),\
 encodeLonLat(133,-10),\
 \
 /* Columbia */\
 encodeLonLat(-82,4),\
 encodeLonLat(-75,13),\
 encodeLonLat(-68,1),\
 encodeLonLat(-70,-4),\
 encodeLonLat(-79,-1),\
 encodeLonLat(-82,4),\
 \
 /* Thailand is not officially 144390 but last time I flew over there, it worked great anyway... */\
 encodeLonLat(98,6),\
 encodeLonLat(97,19),\
 encodeLonLat(100,21),\
 encodeLonLat(107,17),\
 encodeLonLat(103,6),\
 encodeLonLat(98,6),\
 \
 POLYGON_LIST_END\
 };

#define BOUNDARY_144930 {\
 /* Argentina is 144930 by some accounts and 145575 by others... */\
 encodeLonLat(-66,-17),\
 encodeLonLat(-52,-33),\
 encodeLonLat(-68,-51),\
 encodeLonLat(-73,-51),\
 encodeLonLat(-66,-17),\
 \
 POLYGON_LIST_END\
 };

#define BOUNDARY_145525 {\
 /* Thailand is officially 145525 but it may not be real. */\
 encodeLonLat(98,6),\
 encodeLonLat(97,19),\
 encodeLonLat(100,21),\
 encodeLonLat(107,17),\
 encodeLonLat(103,6),\
 encodeLonLat(98,6),\
 \
 POLYGON_LIST_END\
};

#define BOUNDARY_145010 {\
 /* Venezuela. */\
 encodeLonLat(-75,13),\
 encodeLonLat(-60,11),\
 encodeLonLat(-60,4),\
 encodeLonLat(-68,1),\
 encodeLonLat(-75,13),\
 \
 POLYGON_LIST_END\
};

#define BOUNDARY_145175 {\
 /* Australia */\
 encodeLonLat(148,-45),\
 encodeLonLat(114,-36),\
 encodeLonLat(112,-20),\
 encodeLonLat(136,-6),\
 encodeLonLat(156,-26),\
 encodeLonLat(148,-45),\
 \
 POLYGON_LIST_END\
};

#define BOUNDARY_145575 {\
 /* Brazil. */\
 encodeLonLat(-70,-4),\
 encodeLonLat(-68,1),\
 encodeLonLat(-60,4),\
 encodeLonLat(-40,-2),\
 encodeLonLat(-32,-5),\
 encodeLonLat(-38,-20),\
 encodeLonLat(-52,-33),\
 encodeLonLat(-70,-4),\
 /* Argentina */\
 encodeLonLat(-66,-17),\
 encodeLonLat(-52,-33),\
 encodeLonLat(-68,-51),\
 encodeLonLat(-73,-51),\
 encodeLonLat(-66,-17),\
 \
 POLYGON_LIST_END\
};

#define CORE_144660 {\
/* Japanese islands */\
	encodeLonLat(127,27),\
	encodeLonLat(129,27),\
	encodeLonLat(129,25),\
	encodeLonLat(127,25),\
	encodeLonLat(127,27),\
    \
	encodeLonLat(130,33),\
    encodeLonLat(144,44),\
	encodeLonLat(145,44),\
    encodeLonLat(140,34),\
	encodeLonLat(130,31),\
    encodeLonLat(130,31),\
	encodeLonLat(130,33),\
	\
	POLYGON_LIST_END\
};

#define CORE_144620 {\
	encodeLonLat(127,34),\
    encodeLonLat(129,37),\
    encodeLonLat(130,37),\
    encodeLonLat(130,35),\
    encodeLonLat(128,34),\
    encodeLonLat(127,34),\
	\
	POLYGON_LIST_END\
};

#define CORE_144390 {\
	encodeLonLat(-161,21),\
    encodeLonLat(-160,24),\
	encodeLonLat(-154,20),\
	encodeLonLat(-155,18),\
	encodeLonLat(-161,21),\
	\
    encodeLonLat(-116,30),\
	encodeLonLat(-128,51),\
	encodeLonLat(-123,51),\
    encodeLonLat(-113,30),\
	encodeLonLat(-116,30),\
	\
	encodeLonLat(-100,28),\
	encodeLonLat(-94,44),\
	encodeLonLat(-68,45),\
	encodeLonLat(-80,32),\
	encodeLonLat(-100,28),\
	\
	encodeLonLat(-81,32),\
	encodeLonLat(-79,25),\
	encodeLonLat(-82,25),\
	encodeLonLat(-84,31),\
    encodeLonLat(-81,32),\
	\
	/* Thailand */\
	encodeLonLat(98,6),\
	encodeLonLat(97,19),\
	encodeLonLat(100,20),\
	encodeLonLat(106,17),\
	encodeLonLat(101,6),\
	encodeLonLat(98,6),\
	\
	encodeLonLat(110,2),\
	encodeLonLat(111,2),\
	encodeLonLat(111,1),\
	encodeLonLat(110,1),\
	encodeLonLat(110,2),\
	\
    encodeLonLat(112,-9),\
    encodeLonLat(112,-7),\
    encodeLonLat(116,-8),\
    encodeLonLat(116,-9),\
    encodeLonLat(112,-9),\
	\
    /* Guam */\
    encodeLonLat(144,13),\
    encodeLonLat(145,16),\
    encodeLonLat(146,16),\
    encodeLonLat(145,13),\
    encodeLonLat(144,13),\
	\
	POLYGON_LIST_END\
};

#define CORE_144800 {\
  	encodeLonLat(-5,58),\
	encodeLonLat(25,58),\
	encodeLonLat(25,42),\
	encodeLonLat(-5,42),\
	encodeLonLat(-5,58),\
	\
	encodeLonLat(-26,14),\
    encodeLonLat(-26,18),\
    encodeLonLat(-22,17),\
    encodeLonLat(-22,14),\
    encodeLonLat(-26,14),\
	\
    encodeLonLat(-32,41),\
    encodeLonLat(-24,38),\
    encodeLonLat(-24,36),\
    encodeLonLat(-32,39),\
    encodeLonLat(-32,41),\
	\
	encodeLonLat(35,36),\
	encodeLonLat(35,38),\
	encodeLonLat(36,38),\
	encodeLonLat(36,36),\
	encodeLonLat(35,36),\
	\
	encodeLonLat(37,40),\
	encodeLonLat(37,41),\
    encodeLonLat(45,42),\
	encodeLonLat(44,40),\
    encodeLonLat(37,40),\
	\
	encodeLonLat(30,40),\
	encodeLonLat(29,40),\
	encodeLonLat(29,42),\
	encodeLonLat(30,42),\
    encodeLonLat(30,40),\
	\
    /* S Africa */\
	encodeLonLat(18,-33),\
	encodeLonLat(20,-33),\
	encodeLonLat(20,-35),\
	encodeLonLat(18,-35),\
	encodeLonLat(18,-33),\
    /* S Africa */\
	encodeLonLat(27,-25),\
	encodeLonLat(29,-25),\
	encodeLonLat(29,-27),\
    encodeLonLat(27,-27),\
    encodeLonLat(27,-25),\
	\
	POLYGON_LIST_END\
};

#define CORE_144640 {\
   encodeLonLat(88,44),\
   encodeLonLat(88,43),\
   encodeLonLat(87,43),\
   encodeLonLat(87,44),\
   encodeLonLat(88,44),\
   /* Kunming */\
   encodeLonLat(103,24),\
   encodeLonLat(102,24),\
   encodeLonLat(102,25),\
   encodeLonLat(103,25),\
   encodeLonLat(103,24),\
   \
   encodeLonLat(103,31),\
   encodeLonLat(106,31),\
   encodeLonLat(107,29),\
   encodeLonLat(103,29),\
   encodeLonLat(103,31),\
   \
   encodeLonLat(107,26),\
   encodeLonLat(106,26),\
   encodeLonLat(106,27),\
   encodeLonLat(107,27),\
   encodeLonLat(107,26),\
   /* Xian */\
   encodeLonLat(109,34),\
   encodeLonLat(108,34),\
   encodeLonLat(108,35),\
   encodeLonLat(109,35),\
   encodeLonLat(109,34),\
   \
   encodeLonLat(112,32),\
   encodeLonLat(115,40),\
   encodeLonLat(119,40),\
   encodeLonLat(123,32),\
   encodeLonLat(121,28),\
   encodeLonLat(112,32),\
   \
   POLYGON_LIST_END\
};

#define CORE_145175 {\
	/* Australia */\
	encodeLonLat(115,-31),\
	encodeLonLat(118,-31),\
	encodeLonLat(118,-35),\
	encodeLonLat(115,-35),\
	encodeLonLat(115,-31),\
	\
	encodeLonLat(136,-34),\
	encodeLonLat(148,-38),\
	encodeLonLat(147,-44),\
	encodeLonLat(136,-34),\
	\
	encodeLonLat(147,-36),\
	encodeLonLat(152,-26),\
	encodeLonLat(155,-26),\
	encodeLonLat(150,-37),\
	encodeLonLat(147,-36),\
	\
	encodeLonLat(152,-26),\
	encodeLonLat(144,-18),\
	encodeLonLat(147,-18),\
    encodeLonLat(155,-26),\
    encodeLonLat(152,-26),\
	\
    POLYGON_LIST_END\
};

#define CORE_145575 {\
	encodeLonLat(-57,-37),\
	encodeLonLat(-57,-38),\
	encodeLonLat(-58,-38),\
	encodeLonLat(-58,-37),\
	encodeLonLat(-57,-37),\
	\
	encodeLonLat(-62,-38),\
	encodeLonLat(-62,-39),\
	encodeLonLat(-63,-39),\
	encodeLonLat(-63,-38),\
	encodeLonLat(-62,-38),\
	\
    POLYGON_LIST_END\
};

void APRS_checkWorldMapBoundaries_convex_polygons();
void APRS_checkWorldMapCoreAreas_convex_polygons();
void APRS_checkCoreVertexExcursion();

#endif /* INC_WORLDMAP_H_ */

#ifndef	__SUPERCOOLMODELHINCLUDED__
#define	__SUPERCOOLMODELHINCLUDED__
#include "SuperCoolModelFix.h"
//
//	Definitions of data structures for Human Head Studios'
//	Super Cool Model (SCM) format.
//
//	Copyright (C) 2000 Human Head Studios and Serotonin Software
//
//	Modification History
//	--------------------
//
//	Date		Who			Description
//	--------	---			----------------------------------------
//	11/30/00	CJB			Coded based on early source from HH
//	04/02/01	CJB			Corrected order of rotation data in seq.
//	04/18/01	CJB			Added notes on applying sequence data to
//							verts and on special use of joint #1
//


//
//	-------------------------------------------------------
//	SCM Version 3 file specification (Super Cool Model)
//	by Human Head Studios, written down by Chris Burke,
//	based on pre-release code from Paul MacArthur
//	-------------------------------------------------------
//	
//	Name				Size		Description
//	-------------------------------------------------------
//	MagicNumber						BYTE[4]		SCM\0
//	FileVersion						int			3
//	ModelType						int			Flags for model type:
//									MODELTYPE_VERTEX	0		vertex model
//									MODELTYPE_NORMAL	1		skeletal model
//									VECTOR_COMPRESSION	0x10000	vectors compressed 8.8 (OBSOLETE)
//									ROTATOR_COMPRESSION	0x20000	rotators compressed 16 (OBSOLETE)
//									SCALE_COMPRESSION	0x40000	scales compressed 4.4 (OBSOLETE)
//	
//	ResourcePath					char[256]	import path name\0
//	ExportPath						char[256]	export path name\0
//	
//	NumberSkins						UINT		Number of skins for this model
//	<skin definition * NumberSkins>
//		SkinName					char[32]	skin name\0
//		palette						BYTE[768]	skin palette (256 entries, 24 bits)
//		BitmapWidth					int			width of bitmap in pixels (256 max)
//		BitmapHeight				int			height of bitmap in pixels (256 max)
//		Bitmap						BYTE[w*h]	width x height skin
//	
//	NumberMeshes					UINT		number of meshes
//	<mesh definition * NumberMeshes>
//		MeshName					char[32]	mesh name\0
//	
//		NumberTriangles				UINT		number of triangles in this mesh
//		<triangle definition * NumberTriangles>
//			A						USHORT		vertex index a
//			A_S						BYTE		vertex a texture coordinate s
//			A_T						BYTE		vertex a texture coordinate t
//			B						USHORT		vertex index b
//			B_S						BYTE		vertex b texture coordinate s
//			B_T						BYTE		vertex b texture coordinate t
//			C						USHORT		vertex index c
//			C_S						BYTE		vertex c texture coordinate s
//			C_T						BYTE		vertex c texture coordinate t
//			polygroup				BYTE		polygon group (0..15) this triangle belongs to
//	
//		NumberVertices				UINT		number of vertices in this mesh
//		<skeletal vertex definition * NumberVertices>
//			VertOffset1				FVECTOR		Position relative to WeightJoint1:
//												float	x
//												float	y
//												float	z
//			WeightJoint1			BYTE		Index of 1st joint weighted to
//			WeightFactor1			BYTE		Range 0.0 - 1.0 (0 - 255) 
//			VertOffset2				FVECTOR		Position relative to WeightJoint2
//												float	x
//												float	y
//												float	z
//			WeightJoint2			BYTE		Index of 2nd joint weighted to
//			WeightFactor2			BYTE		1.0 - WeightFactor1 (255 - xx)
//	
//		<polygroup definition * 16>
//			SkinPage				USHORT		Index of skin for this group
//			PolyGroupFlags			int			Flags for this polygon group (DWORD)
//									PF_Invisible	0x00000001	Poly is invisible.
//									PF_Masked		0x00000002	Poly should be drawn masked.
//									PF_Translucent	0x00000004	Poly is transparent.
//									PF_NotSolid		0x00000008	Poly is not solid, doesn't block.
//									PF_Environment	0x00000010	Poly should be drawn environment mapped.
//									PF_Semisolid	0x00000020	Poly is semi-solid = collision solid, Csg nonsolid.
//									PF_Modulated	0x00000040	Modulation transparency.
//									PF_FakeBackdrop	0x00000080	Poly looks exactly like backdrop.
//									PF_TwoSided		0x00000100	Poly is visible from both sides.
//									PF_AutoUPan		0x00000200	Automatically pans in U direction.
//									PF_AutoVPan		0x00000400	Automatically pans in V direction.
//									PF_NoSmooth		0x00000800	Don't smooth textures.
//									PF_BigWavy		0x00001000	Poly has a big wavy pattern in it.
//									PF_SmallWavy	0x00002000	Small wavy pattern (for water/enviro reflection).
//									PF_Flat			0x00004000	Flat surface.
//									PF_LowShadowDetail 0x00008000	Low detail shadows.
//									PF_NoMerge		0x00010000	**OVERRIDDEN** Used for LOD BIAS
//									PF_CloudWavy	0x00020000	Polygon appears wavy like clouds.
//									PF_DirtyShadows	0x00040000	Dirty shadows.
//									PF_BrightCorners 0x00080000	Brighten convex corners.
//									PF_SpecialLit	0x00100000	Only speciallit lights apply to this poly.
//									PF_Gouraud		0x00200000	Gouraud shaded.
//									PF_Unlit		0x00400000	Unlit.
//									PF_HighShadowDetail 0x00800000	High detail shadows.
//									PF_Memorized	0x01000000	Editor: Poly is remembered.
//									PF_Selected		0x02000000	Editor: Poly is selected.
//									PF_Portal		0x04000000	Portal between iZones.
//									PF_Mirrored		0x08000000	Reflective surface.
//			DamageJoint				BYTE		Index of damage joint
//			GroupName				char[32]	Name of polygon group\0
//	
//	NumberJoints					INT			Number of joints in file
//	<joint definition * NumberJoints>
//		JointName					char[32]	"Right Elbow"\n
//		Parent						int			index of parent joint or (-1)
//		Children					int[4]		children indeces or (-1)
//		JointGroup					int			joint group enumerated type (jointgroup_e)
//									JOINTGROUP_LEGS		0		below waist
//									JOINTGROUP_TORSO	1		below neck
//									JOINTGROUP_HEAD		2		head
//	
//		flags						int			static joint properties
//									JOINT_FLAG_BLENDJOINT	0x01
//									JOINT_FLAG_ACCELERATIVE	0x02
//									JOINT_FLAG_SPRINGPOINT	0x04
//									JOINT_FLAG_ANCHORED		0x08
//									JOINT_FLAG_COLLISION	0x10
//									JOINT_FLAG_REMOVEABLE	0x20	abs_position
//									JOINT_FLAG_ATTACHPOINT	0x40	abs_rotation
//									JOINT_FLAG_FREEPOINT	0X80
//	
//		<collision box plane defintion * 6>
//			plane					fplane		Defintions of planes of collision box
//												relative to joint coord system. Plane is
//												defined as a normal vector and radius;
//												the plane is perpendicular to the normal
//												at the specified radius along the normal
//												from (0.,0.,0.)
//												float	normalX
//												float	normalY
//												float	normalZ
//												float	pointRadius
//	
//	NumberEvents					UINT
//	<event definition * NumberEvents>
//		EventName					char[64]	Name of the event (first typically "None")
//	
//	NumberSequences					UINT
//	<sequence definition * NumberSequences>
//		SequenceName				char[32]	name of sequence\0
//		FirstFrame					int			starting frame of sequence
//		NumFrames					int			duration of sequence (# frames)
//		LinearVel					float		frame rate of sequence
//		BNormalized					BYTE		Non-zero if sequence is normalized (typically 0?)
//		CompressedLength			int			# BYTEs compressed sequence data
//												(at most 19 BYTEs per joint per frame)
//												(see below for description)
//		Compressed					BYTE[CompressedSize]
//	
//	NumberFrames					UINT
//	<frame definition * NumberFrames>
//		SequenceIndex				short		Index of sequence it's part of
//		eventID						short		Index of associated event (0 if none)
//		bounds						FBox		Bounding box of this frame ??
//												float	ulhcX	?? corner 1 X ??
//												float	ulhxY	?? corner 1 Y ??
//												float	ulhcZ	?? corner 1 Z ??
//												float	lrhcX	?? corner 2 X ??
//												float	lrhcY	?? corner 2 Y ??
//												float	lrhcZ	?? corner 2 Z ??
//												int		i1		?? typically 1 ??
//	
//	CumulativeScale					FVector		?? X, Y, Z scale factors ??
//												float	scaleX
//												float	scaleY
//												float	scaleZ
//	
//	<decimation chunk definition * NumberMeshes>
//		DecCount					int			Entry count
//		DecChunkSize				int			Size of decimation data (may be zero)
//		DecData						BYTE[DecChunkSize]
//	
//	
//	Note: The 16 polygon groups have predefined meanings but it isn't clear that
//	every model uses these meanings, and although they are pretty typical names
//	for bipedal models they don't match the body parts used in the game code of
//	Rune. The predefined meanings are:
//		0	POLYGROUP_NONE			Unspecified
//		1	POLYGROUP_HEAD			Head
//		2	POLYGROUP_TORSO			Torso
//		3	POLYGROUP_LUPPERARM		Left Upper Arm
//		4	POLYGROUP_LLOWERARM		Left Lower Arm
//		5	POLYGROUP_RUPPERARM		Right Upper Arm
//		6	POLYGROUP_RLOWERARM		Right Lower Arm
//		7	POLYGROUP_WAIST			Waist (Hips)
//		8	POLYGROUP_LUPPERLEG		Left Upper Leg
//		9	POLYGROUP_LLOWERLEG		Left Lower Leg
//		10	POLYGROUP_RUPPERLEG		Right Upper Leg
//		11	POLYGROUP_RLOWERLEG		Right Lower Leg
//		12	POLYGROUP_MISC1			Miscellaneous 1
//		13	POLYGROUP_MISC2			Miscellaneous 2
//		14	POLYGROUP_MISC3			Miscellaneous 3
//		15	POLYGROUP_MISC4			Miscellaneous 4
//	
//	
//	-------------------------------------------------------
//	Format of compressed sequence data in an SCM file
//	-------------------------------------------------------
//	
//	Name				Size		Description
//	-------------------------------------------------------
//	AnimMask			BYTE		Indicates which fields are present
//	
//	PosX				int16		X-coordinate of joint root (float 8.8) (relative to parent root, before rotations)
//	PosY				int16		Y-coordinate of joint root (float 8.8) (relative to parent root, before rotations)
//	PosZ				int16		Z-coordinate of joint root (float 8.8) (relative to parent root, before rotations)
//	
//	ScaleX				int16		X scale of joint mesh (float 8.8)
//	ScaleZ				int16		NOTE ORDER! Z scale of joint mesh (float 8.8)
//	ScaleY				int16		NOTE ORDER! Y scale of joint mesh (float 8.8)
//	
//	Pitch				int16		Y-rotation of joint (float 8.8) (heirarchical)
//	Yaw					int16		Z-rotation of joint (float 8.8) (heirarchical)
//	Roll				int16		X-rotation of joint (float 8.8) (heirarchical)
//	
//	NOTE: ROTATIONS ARE MEASURED IN 256THS OF A CIRCLE, NOT RADIANS OR DEGREES.
//	Note: If PosX, PosY, or PosZ is not present, it is assumed to be (0.,0.,0.)
//	Note: If ScaleX, ScaleY, or ScaleZ is not present, it is assumed to be (1.,1.,1.))
//	Note: If RotX, RotY, or RotZ is not present, it is assumed to be (0.,0.,0.)
//	Note: Flag bits in AnimMask are as follows (bit set indicates field present):
//				ANIMMASK_POSX		0x01		PosX is present
//				ANIMMASK_POSY		0x02		PosY is present
//				ANIMMASK_POSZ		0x04		PosZ is present
//				ANIMMASK_SCALEXZ	0x08		ScaleX, then ScaleZ are present
//				ANIMMASK_SCALEY		0x10		ScaleY is present
//				ANIMMASK_PITCH		0x20		Pitch is present
//				ANIMMASK_YAW		0x40		Yaw is present
//				ANIMMASK_ROLL		0x80		Roll is present
//	Only those fields with bits set in AnimMask are present in the data.
//	The fields present are read or written in order of increasing flag bit,
//	e.g. PosX is read or written before PosY.
//	
//	Note: A float is compressed to 8.8 format by multiplying it by 256.0 and 
//	casting the result to (short) - which returns the 16 least-significant bits
//	of the integer part of the scaled float. Consequently, values outside the
//	range +/-127 will be horribly distorted and should not be allowed.
//
//	Note: The data for all joints of the frame are concatenated.
//
//	Note: The sequence data is applied as follows:
//	1)	Set all joint positions to the values given in the corresponding
//		PosX, PosY, PosZ entries
//	2)	Initialize a rotation matrix to the identity matrix
//	3)	Beginning with the root joint and descending heirarchically:
//		a)	Rotate the joint position using the rotation matrix
//		b)	Calculate a new rotation matrix by applying corresponding
//			Pitch, Yaw, and Roll entries.
//		c)	Rotate all verteces weighted to this joint using the new matrix
//		d)	Pass the new matrix down to each child joint, repeating
//			steps (a) - (d) for that joint.
//	4)	Initialize a translation vector to (0,0,0)
//	5)	Beginning with the root joint and descending heirarchically:
//		a)	Translate the joint position using the translation vector
//		b)	Translate all verteces weighted to this joint by the
//			new joint position
//		c)	Pass the new joint position down to each child joint as
//			the new translation vector, repeating steps (a) - (c)
//			for that joint.
//
//	Note: TYPICALLY in .SCM files created by Human Head Studios, two of the
//	joints have special purposes.
//		Joint #0		Always (0,0,0), never has any rotation or translation
//		Joint #1		Incorporates directional positional changes of the
//						overall skeleton into its position and rotation vectors;
//						does not have the same "bone length" from frame to frame
//						of a sequence.
//	All other joints TYPICALLY maintain constant "bone length" and have the same
//	PosX, PosY, and PosZ values for all frames of each sequence. Also, TYPICALLY
//	if the bone length is not constant, no verteces are weighted to the joint
//	at the "root" of the bone (for example, TYPICALLY no verteces are weighted
//	to Joint #0)
//	
//
//	-------------------------------------------------------
//	Format of decimation data in an SCM file
//	-------------------------------------------------------
//	
//	Name				Size		Description
//	-------------------------------------------------------
//	<decimation definition * DecCount>
//		RecOffset		int			0 for 1st record, then negative of offset to start of previous record
//									(which is also the length of the previous record)
//		VFrom			USHORT		Vertex to collapse
//		VTo				USHORT		Vertex collapsed to
//		ConnectList		USHORT		Starts at vertex count, increments by 1 for each record
//		OldConnectList	USHORT		Value of ConnectList for previous record using same vTO
//		NumberRemoved	USHORT		Number of triangles deleted by this decimation
//		NumberRemapped	USHORT		Number of triangles changed by this decimation
//		<change definition * NumberChanged>
//			CTriID		USHORT		Index of triangle to change
//			Corner		USHORT		Index (0..2) of corner of triangle to change
//			OldTexU		BYTE		Old texture U coordinate
//			OldTexV		BYTE		Old texture V coordinate
//			NewTexU		BYTE		New texture U coordinate
//			NewTexV		BYTE		New texture V coordinate
//		<delete definition * NumberDeleted>
//			DTriID		USHORT		Index of triangle to delete
//
//	NOTE: Deleting a triangle DOES NOT change the sequence numbers of other triangles or
//	verteces. All decimation records use the original triangle and vertex sequence numbers
//	(0...original count - 1)
//


//	WARNING: ALL SCM DATA STRUCTURES ARE PACKED TO 1-BYTE BOUNDARIES!!!!
//	*** BUT *** CODE IS CURRENTLY WRITTEN TO READ FIELD-BY-FIELD
//#pragma pack(1)
//#define	__PACKED_SCM_DATA__		1
//	WARNING: ALL SCM DATA STRUCTURES ARE PACKED TO 1-BYTE BOUNDARIES!!!!



#define	kSCM_Magic			"SCM"
#define	kSCM_Version		3

#define	kSCM_NameLen		32
#define	kSCM_MediumNameLen	64
#define	kSCM_LongNameLen	256

//	Treble float type used in original Human Head SCM code
//	If you don't have a C3dVector class, D3DVECTOR will work
//typedef	D3DVECTOR	FVECTOR;
#define	kSCM_FVectorSize	(3*4)
//typedef struct
//{
//	float		x;
//	float		y;
//	float		z;
//}	FVECTOR;
typedef	D3DVECTOR	FVECTOR;

//	Definition of a plane as a normal vector and a distance
//	from (0,0,0) used in original Human Head SCM code
#define	kSCM_FPlaneSize	(kSCM_FVectorSize + 4)
typedef struct
{
	FVECTOR		normal;					//	Direction from (0,0,0) of plane's normal
	float		distance;				//	Distance along normal vector from (0,0,0) to point on plane
}	FPLANE;


//	Bounding box definition used in original Human Head SCM code
#define	kSCM_FBoxSize	(kSCM_FVectorSize + kSCM_FVectorSize + 4)
typedef struct
{
	FVECTOR		ulc;					//	Upper left corner
	FVECTOR		lrc;					//	Lower right corner
	int			i;						//	unknown; typically 1
}	FBOX;


//	Values for ModelType field
enum
{
	MODELTYPE_VERTEX = 0,			//	vertex model
	MODELTYPE_NORMAL = 1,			//	skeletal model
	VECTOR_COMPRESSION = 0x00010000,	//	vectors compressed 8.8 (OBSOLETE)
	ROTATOR_COMPRESSION = 0x00020000,	//	rotators compressed 16 (OBSOLETE)
	SCALE_COMPRESSION = 0x00040000,	//	scales compressed 4.4 (OBSOLETE)
	MODELTYPE_MASK = 0x0000FFFF	//	mask for model type field
};


//	Standard numbering for the 16 polygon groups
//	NOTE: These polygroup definitions don't match the bodyparts the game uses
enum
{
	POLYGROUP_NONE,
	POLYGROUP_HEAD,
	POLYGROUP_TORSO,
	POLYGROUP_LUPPERARM,
	POLYGROUP_LLOWERARM,
	POLYGROUP_RUPPERARM,
	POLYGROUP_RLOWERARM,
	POLYGROUP_WAIST,
	POLYGROUP_LUPPERLEG,
	POLYGROUP_LLOWERLEG,
	POLYGROUP_RUPPERLEG,
	POLYGROUP_RLOWERLEG,
	POLYGROUP_MISC1,
	POLYGROUP_MISC2,
	POLYGROUP_MISC3,
	POLYGROUP_MISC4
};


//	Values for the PolyGroupFlags field
enum
{
	// Regular in-game flags.
	PF_Invisible = 0x00000001,	// Poly is invisible.
	PF_Masked = 0x00000002,	// Poly should be drawn masked.
	PF_Translucent = 0x00000004,	// Poly is transparent.
	PF_NotSolid = 0x00000008,	// Poly is not solid, doesn't block.
	PF_Environment = 0x00000010,	// Poly should be drawn environment mapped.
	PF_Semisolid = 0x00000020,	// Poly is semi-solid = collision solid, Csg nonsolid.
	PF_Modulated = 0x00000040,	// Modulation transparency.
	PF_FakeBackdrop = 0x00000080,	// Poly looks exactly like backdrop.
	PF_TwoSided = 0x00000100,	// Poly is visible from both sides.
	PF_AutoUPan = 0x00000200,	// Automatically pans in U direction.
	PF_AutoVPan = 0x00000400,	// Automatically pans in V direction.
	PF_NoSmooth = 0x00000800,	// Don't smooth textures.
	PF_BigWavy = 0x00001000,	// Poly has a big wavy pattern in it.
	PF_SmallWavy = 0x00002000,	// Small wavy pattern (for water/enviro reflection).
	PF_Flat = 0x00004000,	// Flat surface.
	PF_LowShadowDetail = 0x00008000,	// Low detail shadows.
	PF_NoMerge = 0x00010000,	// **OVERRIDDEN** Used for LOD BIAS
	PF_CloudWavy = 0x00020000,	// Polygon appears wavy like clouds.
	PF_DirtyShadows = 0x00040000,	// Dirty shadows.
	PF_BrightCorners = 0x00080000,	// Brighten convex corners.
	PF_SpecialLit = 0x00100000,	// Only speciallit lights apply to this poly.
	PF_Gouraud = 0x00200000,	// Gouraud shaded.
	PF_Unlit = 0x00400000,	// Unlit.
	PF_HighShadowDetail = 0x00800000,	// High detail shadows.
	PF_Memorized = 0x01000000,	// Editor: Poly is remembered.
	PF_Selected = 0x02000000,	// Editor: Poly is selected.
	PF_Portal = 0x04000000,	// Portal between iZones.
	PF_Mirrored = 0x08000000,	// Reflective surface.
};


//	Values for the flags field (in scmjoint_t)
enum
{
	JOINT_FLAG_BLENDJOINT = 0x01,
	JOINT_FLAG_ACCELERATIVE = 0x02,
	JOINT_FLAG_SPRINGPOINT = 0x04,
	JOINT_FLAG_ANCHORED = 0x08,
	JOINT_FLAG_COLLISION = 0x10,
	JOINT_FLAG_REMOVEABLE = 0x20,		// abs_position
	JOINT_FLAG_ATTACHPOINT = 0x40,		// abs_rotation
	JOINT_FLAG_FREEPOINT = 0X80
};


//	Values for the JointGroup field
enum
{
	JOINTGROUP_LEGS = 0,			//	below waist
	JOINTGROUP_TORSO = 1,			//	below neck
	JOINTGROUP_HEAD = 2				//	head
};


//	Values for the AnimMask field
enum
{
	ANIMMASK_POSX = 0x01,
	ANIMMASK_POSY = 0x02,
	ANIMMASK_POSZ = 0x04,
	ANIMMASK_SCALEXZ = 0x08,
	ANIMMASK_SCALEY = 0x10,
	ANIMMASK_PITCH = 0x20,
	ANIMMASK_YAW = 0x40,
	ANIMMASK_ROLL = 0x80
};


//	Basic file header
#define	kSCM_HeaderSize	(4 + 4 + 4 + kSCM_LongNameLen + kSCM_LongNameLen)
typedef struct
{
	char		MagicNumber[4];					//	Magic Number "SCM\0"
	int			FileVersion;					//	Generally 3
	int			ModelType;						//	Flags for model type:
	char		ResourcePath[kSCM_LongNameLen];	//	import path name\0
	char		ExportPath[kSCM_LongNameLen];	//	export path name\0
}	scmheader_t;


//	Skin section header
typedef struct
{
	int			NumberSkins;			//	Number of skins for this mesh
}	scmskinhdr_t;

//	Header for each skin
#define	kSCM_SkinSubhdrSize	(kSCM_NameLen + 768 + 4 + 4 + 1)
typedef struct
{
	char		SkinName[kSCM_NameLen];	//	skin name\0
	BYTE		palette[768];			//	skin palette (256 entries, 24 bits 8-8-8 RGB)
	int			BitmapWidth;			//	width of bitmap in pixels (256 max)
	int			BitmapHeight;			//	height of bitmap in pixels (256 max)
	BYTE		Bitmap[1];				//	BYTE[w*h]	width x height skin
	//
}	scmskin_t;


//	Mesh section header
typedef struct
{
	int			NumberMeshes;			//	Number of discrete vertex frames
}	scmmeshhdr_t;

//	Mesh name header
#define	kSCM_MeshNameSize (kSCM_NameLen)
typedef struct
{
	char		MeshName[kSCM_NameLen];	//	mesh name\0
}	scmmeshname_t;


//	Triangle subsection header
typedef struct
{
	int			NumberTriangles;		//	Number of triangles in this mesh
}	scmtrianglehdr_t;

//	Triangle definition
//
//	WARNING: This structure IS NOT a multiple of 4 BYTEs in size;
//	must be read field-by-field from file.
#define	kSCM_TriSize	(2+1+1+2+1+1+2+1+1+1)
typedef struct
{
	unsigned short	A;					//	vertex index a
	BYTE			A_S;				//	vertex a texture coordinate s
	BYTE			A_T;				//	vertex a texture coordinate t
	unsigned short	B;					//	vertex index b
	BYTE			B_S;				//	vertex b texture coordinate s
	BYTE			B_T;				//	vertex b texture coordinate t
	unsigned short	C;					//	vertex index c
	BYTE			C_S;				//	vertex c texture coordinate s
	BYTE			C_T;				//	vertex c texture coordinate t
	BYTE			polygroup;			//	polygon group (0..15) this triangle belongs to
	//	Ideally the structure would be 3 BYTEs larger here
	//	to make it align on a 32-bit boundary. But, no such luck.
	//	kSCM_TriSize is the size in BYTEs of the file data.
}	scmtriangle_t;


//	Vertex subsection header
typedef struct
{
	int			NumberVertices;			//	Number of vertices in this mesh
}	scmvertexhdr_t;

//	Vertex definition
//
//	WARNING: Fields in this structure ARE NOT aligned on multiples
//	of 4 BYTEs; must be read field-by-field from file.
#define	kSCM_VertSize	(12+1+1+12+1+1)
typedef struct
{
	FVECTOR		VertOffset1;			//	Position relative to WeightJoint1:
	BYTE		WeightJoint1;			//	Index of 1st joint weighted to
	BYTE		WeightFactor1;			//	?? Range 0.0 - 1.0 ??
	//	Ideally there would be a 2-BYTE item here for alignment, BUT NO!
	FVECTOR		VertOffset2;			//	Position relative to WeightJoint2:
	BYTE		WeightJoint2;			//	Index of 2nd joint weighted to
	BYTE		WeightFactor2;			//	?? Range 0.0 - 1.0 ??
	//	Ideally the structure would be 2 BYTEs larger here
	//	to make it align on a 32-bit boundary. But, no such luck.
	//	kSCM_VertSize is the size in BYTEs of the file data.
}	scmvertex_t;


//	Polygon group definitions. There are always 16 of these in
//	the polygon group subsection.
//
//	WARNING: Fields in this structure ARE NOT aligned on multiples
//	of 4 BYTEs; must be read field-by-field from file.
#define	kSCM_NumPolyGroups	16
#define	kSCM_PGSize		(2+4+1+32)
typedef struct
{
	unsigned short	SkinPage;					//	Index of skin for this group
	//	Ideally there would be a 2-BYTE item here for alignment, BUT NO!
	int				PolyGroupFlags;				//	Flags for this polygon group (DWORD)
	BYTE			DamageJoint;				//	Index of damage joint
	char			GroupName[kSCM_NameLen];	//	Name of polygon group\0
	//	Ideally the structure would be 3 BYTEs larger here
	//	to make it align on a 32-bit boundary. But, no such luck.
	//	kSCM_PGSize is the size in BYTEs of the file data.
}	scmpolygroup_t;


//	Joint subsection header
typedef struct
{
	int			NumberJoints;					//	Number of joints in file
}	scmjointhdr_t;

//	Joint definition
#define	kSCM_NoJoint	(-1)
#define	kSCM_MaxChildJoints	4
#define	kSCM_JointSize	(kSCM_NameLen + 4 + 4*kSCM_MaxChildJoints + 4 + 4 + 6*kSCM_FPlaneSize)
typedef struct
{
	char		JointName[kSCM_NameLen];		//	"Right Elbow"\n
	int			Parent;							//	index of parent joint or (-1)
	int			Children[kSCM_MaxChildJoints];	//	children indeces or (-1)
	int			JointGroup;						//	joint group enumerated type (jointgroup_e)
	int			flags;							//	static joint properties
	FPLANE		CollisionBox[6];				//	Defintions of planes of collision box
	//	relative to joint coord system.
}	scmjoint_t;


//	Event subsection header
typedef struct
{
	int			NumberEvents;					//	Number of events in file
}	scmeventhdr_t;

//	Event definition
#define	kSCM_EventSize	(kSCM_MediumNameLen)
typedef struct
{
	char		EventName[kSCM_MediumNameLen];	//	?? name of the event (first typically "None")
}	scmevent_t;


//	Sequence subsection header
typedef struct
{
	int			NumberSequences;				//	Number of sequences in file
}	scmsequencehdr_t;

//	Sequence definition
//
//	WARNING: Fields in this structure ARE NOT aligned on multiples
//	of 4 BYTEs; must be read field-by-field from file.
#define	kSCM_NoFrameID					(-1)
#define	kSCM_SequenceSubhdrSize			(kSCM_NameLen + 4 + 4 + 4 + 1 + 4 + 1)
#define	kSCM_MaxCompressedDataPerJoint	19
typedef struct
{
	char		SequenceName[kSCM_NameLen];		//	name of sequence\0
	int			FirstFrame;						//	starting frame of sequence
	int			NumFrames;						//	duration of sequence (# frames)
	float		LinearVel;						//	frame rate of sequence
	BYTE		bNormalized;					//	Non-zero if sequence is normalized (typically 0?)
	//	Ideally there would be a 3-BYTE item here for alignment, BUT NO!
	int			CompressedLength;				//	# BYTEs compressed sequence data
	//	(at most 19 BYTEs per joint per frame)
	BYTE		Compressed[1];					//	Compressed sequence data, BYTE[CompressedSize]
	//
}	scmsequence_t;

//	Level 1 decompressed version of sequence data.
//	WARNING: This is stored compressed in the file. A 1-BYTE AnimMask field
//	procedes each entry, indicating which fields are present in the entry.
//	WARNING: The ScaleZ and ScaleY fields are interchanged in the file - 
//	see description of in-file data format for more info.
#define	kSCM_AngToDegrees	(360./256.)
typedef struct
{
	//	AnimMask			BYTE		Indicates which fields are present
	short		PosX;							//	X-coordinate of joint root (compressed float 8.8)
	short		PosY;							//	Y-coordinate of joint root (compressedfloat 8.8)
	short		PosZ;							//	Z-coordinate of joint root (compressedfloat 8.8)
	short		ScaleX;							//	X scale of joint mesh (compressedfloat 8.8)
	short		ScaleZ;							//	NOTE ORDER! Z scale of joint mesh (compressedfloat 8.8)
	short		ScaleY;							//	NOTE ORDER! Y scale of joint mesh (compressedfloat 8.8)
	short		Pitch;							//	Neg. rotation of joint about Y axis (compressed float 8.8, 256ths of circle)
	short		Yaw;							//	Neg. rotation of joint about Z axis (")
	short		Roll;							//	Neg. rotation of joint about X axix (")
}	scmseqdataL1_t;

//	Level 2 decompressed version of sequence data.
//	NOTE: The data in the file looks nothing like this, but "means" this.
//	This format is twice as big as scmseqdataL1_t, but much faster to access.
typedef struct
{
	FVECTOR		pos;							//	position of "bone" root joint
	FVECTOR		scale;							//	scale factor for "bone" mesh
	FVECTOR		rot;							//	rotation of mesh around root joint
}	scmseqdata_t;


//	Frames subsection header
typedef struct
{
	int			NumberFrames;					//	Number of frames in file
}	scmframehdr_t;

//	Frame definition
#define	kSCM_FrameSize	(2 + 2 + kSCM_FBoxSize)
typedef struct
{
	short		SequenceIndex;					//	Index of sequence it's part of
	short		eventID;						//	Index of associated event (0 if none)
	FBOX		bounds;							//	Bounding box of this frame ??
}	scmframe_t;


//	Cumulative scale definition
//	NOTE: Not sure what this does
#define	kSCM_CumulativeSize	(kSCM_FVectorSize)
typedef struct
{
	FVECTOR		CumulativeScale;				//	?? X, Y, Z scale factors ??
}	scmcumulativescale_t;


//	Decimation data definition.
//	NOTE: There is one definition per mesh. This is likely level-of-detail information
#define	kSCM_DecimationSubhdrSize	(4 + 4 + 1)
typedef struct
{
	int		DecCount;							//	?? entry count ??
	int		DecChunkSize;						//	Size of decimation data (may be zero)
	BYTE	DecData[1];							//	Decimation data - BYTE[DecChunkSize]
	//
}	scmdecimation_t;


//	Decimation triangle change record
typedef struct
{
	short	CTriID;								//	Index of triangle to change
	short	Corner;								//	Index (0..2) of corner of triangle to change
	BYTE	OldTexU;							//	Old texture U coordinate
	BYTE	OldTexV;							//	Old texture V coordinate
	BYTE	NewTexU;							//	New texture U coordinate
	BYTE	NewTexV;							//	New texture V coordinate
}	scmdectrichangerecord_t;

//	Decimation triangle delete record
typedef struct
{
	short	DTriID;								//	Index of triangle to delete
}	scmdectrideleterecord_t;

//	Decimation record header.
//	Note that DecData is a series of these records, and that each
//	record is variable length
#define	kSCM_DecRecHdrSize	(4+2+2+2+2+2+2)
#define	kSCM_DecRecMapSize	(2+2+1+1+1+1)
#define	kSCM_DecRecDelSize	(2)
typedef struct
{
	int		RecOffset;							//	0 for 1st record, then size of previous record (used as offset to walk list backward)
	short	VFrom;								//	Vertex to collapse
	short	VTo;								//	Vertex collapsed to
	short	ConnectList;						//	??? Starts at vertex count, increments by 1 for each record
	short	OldConnectList;						//	??? Vertex ID to substitute into triangle data (generally same as vTo)
	short	NumberRemoved;						//	Number of triangles deleted by this decimation
	short	NumberRemapped;						//	Number of triangles changed by this decimation
	scmdectrichangerecord_t	Changes[1];			//	Array of NumberRemapped records
	//	after the changed records, there's an array of NumberRemoved deleted records
}	scmdecrecord_t;


//	WARNING: ALL SCM DATA STRUCTURES ARE PACKED TO 1-BYTE BOUNDARIES!!!!
//	*** BUT *** CODE IS CURRENTLY WRITTEN TO READ FIELD-BY-FIELD
//#pragma pack()
//	WARNING: ALL SCM DATA STRUCTURES ARE PACKED TO 1-BYTE BOUNDARIES!!!!


#endif

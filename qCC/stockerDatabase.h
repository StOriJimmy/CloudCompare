
#ifndef __STOCKER_DATABASE_HEADER__
#define __STOCKER_DATABASE_HEADER__

#include "ccHObject.h"
#include "StBlock.h"
#include "StBlockGroup.h"
#include "StBuilding.h"
#include "StFootPrint.h"
#include "StModel.h"
#include "StPrimGroup.h"

#include "buildercore/StBuilder.h"
#include "BlockDBaseIO.h"

#define MAX_BUILDING_NUMBER 99999999

#define BDDB_PLANESEG_PREFIX		"Plane"
#define BDDB_BOUNDARY_PREFIX		"Boundary"
#define BDDB_INTERSECT_PREFIX		"Intersection"
#define BDDB_OUTLINE_PREFIX			"Outline"
#define BDDB_IMAGELINE_PREFIX		"Imageline"
#define BDDB_PLANEFRAME_PREFIX		"Frame"
#define BDDB_FOOTPRINT_PREFIX		"Footprint"
#define BDDB_BDFOOTPRINT_PREFIX		"BDfootprint"
#define BDDB_BLOCK_PREFIX			"Block"
#define BDDB_LOD1MODEL_PREFIX		"LoD1_"
#define BDDB_LOD2MODEL_PREFIX		"LoD2_"
#define BDDB_LOD3MODEL_PREFIX		"LoD3_"
#define BDDB_TODOPOINT_PREFIX		"TodoPoint"
#define BDDB_TODOLINE_PREFIX		"TodoLine"
#define BDDB_DEDUCED_PREFIX			"Deduced"

#define BDDB_BUILDING_PREFIX		"bd"

#define BDDB_CAMERA_SUFFIX			".camera"
#define BDDB_ORIGIN_CLOUD_SUFFIX	".original"
#define BDDB_PRIMITIVE_SUFFIX		".primitive"
#define BDDB_POLYFITHYPO_SUFFIX		".hypothesis"
#define BDDB_POLYFITOPTM_SUFFIX		".optimized"
#define BDDB_FINALMODEL_SUFFIX		".model"
#define BDDB_IMAGELINE_SUFFIX		".imageline"
#define BDDB_BLOCKGROUP_SUFFIX		".block"
#define BDDB_LOD1MODEL_SUFFIX		".lod1.model"
#define BDDB_LOD2MODEL_SUFFIX		".lod2.model"
#define BDDB_LOD3MODEL_SUFFIX		".lod3.model"
#define BDDB_TODOGROUP_SUFFIX		".todo"

enum importDataType
{
	IMPORT_POINTS,
	IMPORT_IMAGES,
	IMPORT_MISCS,
	IMPORT_MODELS,
	IMPORT_POSTGIS,
	IMPORT_TYPE_END,
};

// https://desktop.arcgis.com/zh-cn/arcmap/10.3/manage-data/las-dataset/lidar-point-classification.htm
namespace LAS_LABEL {
	enum LABEL_TYPE
	{
		Unused,
		Unassigned,
		Ground,
		LowVegetation,
		MedianVegetation,
		HighVegetation,
		Building,
		Noise,
		ModelKey,
		Water,
		Rail,
		RoadSurface,
		Overlap,
		WireGuard,
		WireConductor,
		TransmissionTower,
		WireConnector,
		BridgeDeck,
		HighNoise,
		LABEL_END,
	};
	enum LABEL_SIMPLE
	{

	};
	static const char* g_strLabelName[] = { 
		"none","unassigned","ground","low vegetation","median vegetation","high vegetation",
		"building","noise","modelKey","water","rail","road surface",
		"overlap","wire-guard","wire-conductor","transmission tower","wire-connector","bridge deck",
		"high noise"};

	static const double g_classification_color[] =
	{
		184,184,184,//0
		173,173,173,//1
		167,112,1,	//2
		36,	115,0,	//3
		73,	231,6,	//4
		207,245,123,//5
		215,101,62,	//6
		228,2,	0,	//7
		168,110,2,	//8
		2,	87,	237,//9
		232,227,2,	//10
		231,230,1,	//11
		201,3,	255,//12
		234,226,3,	//13
		232,227,2,	//14
		231,229,0,	//15
		229,226,3,	//16
		230,227,10,	//17
		232,228,1	//18
	};
}

Q_DECLARE_METATYPE(BlockDB::blkDataInfo*)
Q_DECLARE_METATYPE(BlockDB::blkCameraInfo)

#define BLK_DATA_METAKEY "BlkDataInfo"

#define PtCld_Dir_NAME "pointClouds"
#define IMAGE_Dir_NAME "images"
#define MISCS_Dir_NAME "miscs"
#define PRODS_Dir_NAME "products"

class DataBaseHObject : public BDBaseHObject_
{
public:
	DataBaseHObject(QString name = QString()) 
		: BDBaseHObject_(name)
		, m_blkData(new BlockDB::BlockDBaseIO)
	{
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	DataBaseHObject(const StHObject& s) 
		: BDBaseHObject_(s) 
		, m_blkData(new BlockDB::BlockDBaseIO) 
	{
		setPath(s.getPath());
		setDBSourceType(CC_TYPES::DB_MAINDB);
	}
	~DataBaseHObject() {
		if (m_blkData) { delete m_blkData; m_blkData = nullptr; }
	}
	virtual inline void setPath(const QString& tp) override;

	using Container = std::vector<DataBaseHObject *>;

public:
	StHObject* getPointCloudGroup();
	StHObject* getImagesGroup();
	StHObject* getMiscsGroup();
	StHObject* getProductGroup();
	StHObject* getProductItem(QString name);
	StHObject* getProductFiltered();
	StHObject* getProductClassified();
	StHObject* getProductSegmented();
	StHObject* getProductModels();

	static DataBaseHObject* Create(QString absolute_path);
	bool addData(StHObject* obj, BlockDB::blkDataInfo* info, bool exist_info);
	bool addDataExist(BlockDB::blkDataInfo* info);
	void clear();
	bool load();
	bool save();
	bool parseResults(BlockDB::BLOCK_TASK_ID task_id, QStringList results, int copy_mode);
	bool retrieveResults(BlockDB::BLOCK_TASK_ID task_id);
	// copy mode, 0 - copy, 1 - move, 2 - use the origin path
	bool retrieveResults(BlockDB::BLOCK_TASK_ID task_id, QStringList results, int copy_mode);

	BlockDB::BlockDBaseIO* m_blkData;
};

StHObject * createObjectFromBlkDataInfo(BlockDB::blkDataInfo * info, bool return_scene = false);

class BDBaseHObject : public BDBaseHObject_
{
public:
	BDBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_BUILDING);
	}
	BDBaseHObject(const StHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_BUILDING);
	}
	~BDBaseHObject() {}

	using Container = std::vector<BDBaseHObject *>;

public:
	stocker::Vec3d global_shift;
	double global_scale;

	std::vector<stocker::ImageUnit> image_data;
	std::vector<stocker::BuildUnit> build_data;
	stocker::BuilderOption	m_options;

public:	

	StBuilding* GetBuildingGroup(QString building_name, bool check_enable);
	ccPointCloud* GetOriginPointCloud(QString building_name, bool check_enable);
	StPrimGroup* GetPrimitiveGroup(QString building_name);
	StBlockGroup * GetBlockGroup(QString building_name);
	StPrimGroup * GetHypothesisGroup(QString building_name);
	
	StHObject* GetTodoGroup(QString building_name);
	ccPointCloud* GetTodoPoint(QString buildig_name);
	ccPointCloud* GetTodoLine(QString buildig_name);
	stocker::Vec3d ToLocal(stocker::Vec3d pt) { return (pt + global_shift)*global_scale; }
	stocker::Vec3d ToGlobal(stocker::Vec3d pt) { return pt / global_scale - global_shift; }

public:
	//! file path

	std::string GetPathModelObj(std::string building_name);

	stocker::BuildUnit* GetBuildingSp(std::string building_name);
	std::vector<stocker::ImageUnit> GetImageData();

public:
	bool updateBuildUnits(bool use_existing_info);
};

class BDImageBaseHObject : public BDBaseHObject_
{
public:
	BDImageBaseHObject(QString name = QString()) :
		BDBaseHObject_(name) {
		setDBSourceType(CC_TYPES::DB_IMAGE);
	}
	BDImageBaseHObject(const StHObject& s) :
		BDBaseHObject_(s) {
		setDBSourceType(CC_TYPES::DB_IMAGE);
	}
	~BDImageBaseHObject() {}

	using Container = std::vector<BDImageBaseHObject *>;
};

inline bool isDatabaseProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_MAINDB;
}
inline DataBaseHObject* ToDatabaseProject(StHObject* object) {
	return isDatabaseProject(object) ? static_cast<DataBaseHObject*>(object) : nullptr;
}
inline bool isBuildingProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_BUILDING;
}
inline BDBaseHObject* ToBuildingProject(StHObject* object) {
	return isBuildingProject(object) ? static_cast<BDBaseHObject*>(object) : nullptr;
}
inline bool isImageProject(StHObject* object) {
	return object && object->isA(CC_TYPES::ST_PROJECT) && object->getDBSourceType() == CC_TYPES::DB_IMAGE;
}
inline BDImageBaseHObject* ToImageProject(StHObject* object) {
	return isImageProject(object) ? static_cast<BDImageBaseHObject*>(object) : nullptr;
}

QString getCompleteBaseName(QString name);

QString getPrimPathByCloudPath(QString cloud_path);

QString getPrimGroupNameByCloudName(QString cloud_name);

QString getBlockPathByCloudPath(QString cloud_path);

DataBaseHObject* GetRootDataBase(StHObject* obj);
BDBaseHObject* GetRootBDBase(StHObject* obj);
BDImageBaseHObject* GetRootImageBase(StHObject* obj);

StHObject* getChildGroupByName(StHObject* group, QString name, bool auto_create = false, bool add_to_db = false, bool keep_dir_hier = false);

StHObject * findChildByName(StHObject * parent, bool recursive, QString filter, bool strict, CC_CLASS_ENUM type_filter = CC_TYPES::OBJECT, bool auto_create = false, ccGenericGLDisplay * inDisplay = 0);

int GetNumberExcludePrefix(StHObject * obj, QString prefix, CC_CLASS_ENUM type = CC_TYPES::OBJECT);

inline QString BuildingNameByNumber(int number) {
	char name[256];
	sprintf(name, "%s%08d", BDDB_BUILDING_PREFIX, number);
	return name;
}

//! return -1 if no child exists
int GetMaxNumberExcludeChildPrefix(StHObject * obj, QString prefix, CC_CLASS_ENUM type = CC_TYPES::OBJECT);

QString GetNextChildName(StHObject* parent, QString prefix, CC_CLASS_ENUM type = CC_TYPES::OBJECT);

bool StCreatDir(QString dir);

// return new result files, if force success, will return all the existing files whatever the files are successfully moved or not
QStringList moveFilesToDir(QStringList list, QString dir, bool remove_old, QStringList* failed_files = nullptr, bool force_success = false);

inline QStringList _splitStringQ(char* str, const char* seps)
{
	QStringList sub_strs;
	char* token = strtok(str, seps);
	while (token) {
		std::string sub(token);
		sub = sub.substr(0, sub.find_last_of("\t\r\n"));
		sub_strs << QString::fromStdString(sub);
		token = strtok(NULL, seps);
	}
	return sub_strs;
}
#endif
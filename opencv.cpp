#include "pch.h"
#include "OmitProcess.h"
#include "HalconMath.h"


//enum class RCP_OMIT_PROCESS
//{
//	DILATION,
//	DILATION_STDEV,
//	INSP_ROI_MAX_THRESHOLD,
//	INSP_ROI_MIN_THRESHOLD,
//	ORIGINAL_THRESHOLD,
//	PTN_BTH,
//	PTN_WTH,
//	SHARED_BOLB_MIN_AREA,
//	SKELETON_POINTS,
//	SKELETON_POINTS_MIN,
//	STDEV_FILTER_MASK_SIZE,
//	STDEV_FILTER_SCALE,
//	STDEV_THRESHOLD,
//	STICKER_AREA_MAX,
//	STICKER_AREA_MIN,
//	STICKER_CLOSING,
//	STICKER_DILATION,
//	STICKER_HOLES_NUM_MIN,
//	WEAKSC_FLAG,
//	WEAKSC_LOWER_TH,
//	WEAKSC_MIN_LENGTH_FINAL,
//	WEAKSC_MIN_LENGTH_SPLIT,
//	WEAKSC_MIN_UNION_LENGTH,
//	WEAKSC_MIN_UNION_MAXSHIFT,
//	WEAKSC_MIN_UNION_RADIAN,
//	WEAKSC_RESIZE,
//	WEAKSC_UPPER_TH,
//};

HObject COmitProcess::Run(Json::Value a_recipe, HObject ho_Image_Input, HObject a_ho_selected_region, HObject *a_ho_STDEVRegionOutput)
{
	HObject ho_result;

	HObject ho_StdevImage = GetStdDevImage(a_recipe, ho_Image_Input);
	//WriteImage(ho_StdevImage, "jpg", 0, "D:\\ho_StdevImage");

	HObject ho_StdevFilterImage = GetStdDevFilter(a_recipe, ho_Image_Input, ho_StdevImage);
	*a_ho_STDEVRegionOutput = GetStdDevFilterFillup(a_recipe,ho_StdevFilterImage);
	//WriteRegion(*a_ho_STDEVRegionOutput, "D:\\a_ho_STDEVRegionOutput");

	ho_result = GetEnhanceOmitFilter(a_recipe, ho_Image_Input, a_ho_selected_region, ho_StdevImage, *a_ho_STDEVRegionOutput);

	//ho_StdevImage.Clear();

	return ho_result;
}


HObject COmitProcess::GetDefaultRegion(HObject ho_Image_Input, double a_threshold_lower, double a_threshold_upper)
{
	HObject ho_RegionOutput;
	HObject ho_BinaryImageB, ho_BinaryImageW;

	Threshold(ho_Image_Input, &ho_BinaryImageB, 0, a_threshold_lower);
	Threshold(ho_Image_Input, &ho_BinaryImageW, a_threshold_upper, 255);

	ConcatObj(ho_BinaryImageB, ho_BinaryImageW, &ho_RegionOutput);
	FillUp(ho_RegionOutput, &ho_RegionOutput);

	return ho_RegionOutput;
}

HObject COmitProcess::GetStdDevImage(Json::Value a_recipe, HObject ho_Image_Input)
{
	HObject ho_STDEVImg, ho_stdev_scaled_image;

	int mark_filter_size = a_recipe[RCP_OMIT_PROCESS::STDEV_FILTER_MASK_SIZE].asInt();
	DeviationImage(ho_Image_Input, &ho_STDEVImg, mark_filter_size, mark_filter_size);

	int filter_scale = a_recipe[RCP_OMIT_PROCESS::STDEV_FILTER_SCALE].asInt();
	ScaleImage(ho_STDEVImg, &ho_stdev_scaled_image, filter_scale, 0);

	//ho_STDEVImg.Clear();

	return ho_stdev_scaled_image;
}

HObject COmitProcess::GetStdDevFilter(Json::Value a_recipe, HObject a_ho_Image_Input, HObject a_ho_stdev_image)
{
	HObject ho_RegionOutput, ho_Omit_STDEV_Copy, ho_Omit_STDEV, ho_Omit_Th;

	double ptn_threshold_upper = a_recipe[RCP_OMIT_PROCESS::STDEV_THRESHOLD].asDouble();
	Threshold(a_ho_stdev_image, &ho_Omit_STDEV_Copy, ptn_threshold_upper, 255);
	Connection(ho_Omit_STDEV_Copy, &ho_Omit_STDEV_Copy);
	SelectShape(ho_Omit_STDEV_Copy, &ho_Omit_STDEV, "area", "and", 3, 99999999);

	double stddev_dilation_value = a_recipe[RCP_OMIT_PROCESS::DILATION_STDEV].asDouble();
	if (stddev_dilation_value < 0)
		ErosionCircle(ho_Omit_STDEV, &ho_Omit_STDEV, -stddev_dilation_value);
	else
		DilationCircle(ho_Omit_STDEV, &ho_Omit_STDEV, stddev_dilation_value);

	Threshold(a_ho_Image_Input, &ho_Omit_Th, a_recipe[RCP_OMIT_PROCESS::ORIGINAL_THRESHOLD].asDouble(), 255);
	Connection(ho_Omit_Th, &ho_Omit_Th);
	SelectShape(ho_Omit_Th, &ho_Omit_Th, "area", "and", 3, 99999999);

	double dilation_value = a_recipe[RCP_OMIT_PROCESS::DILATION].asDouble();
	DilationCircle(ho_Omit_Th, &ho_Omit_Th, dilation_value);

	ConcatObj(ho_Omit_STDEV, ho_Omit_Th, &ho_RegionOutput);

	//ho_Omit_STDEV_Copy.Clear();
	//ho_Omit_STDEV.Clear();
	//ho_Omit_Th.Clear();

	return ho_RegionOutput;
}

HObject COmitProcess::GetStdDevFilterFillup(Json::Value a_recipe, HObject a_ho_region_Input)
{
	HObject ho_STDEVRegionOutput;

	int fillup_mode = a_recipe[RCP_OMIT_PROCESS::FILLUP_MODE].asInt();

	if (fillup_mode == 0)
		FillUp(a_ho_region_Input, &ho_STDEVRegionOutput);
	else 
	{
		int fillup_area = a_recipe[RCP_OMIT_PROCESS::FILLUP_AREA].asInt();			
		FillUpShape(a_ho_region_Input, &ho_STDEVRegionOutput, "area", 1, fillup_area);
	}
	return ho_STDEVRegionOutput;
}

HObject COmitProcess::GetEnhanceOmitFilter(Json::Value a_recipe, HObject a_ho_image_input, HObject a_ho_region_total_roi, HObject a_ho_stdev_image, HObject ho_STDEVRegionOutput)
{
	//--------------------------------------------------------------------------------------------------------------------------------------
	// Weak Scratch 검출
	HObject ho_ReducedImg, ho_ResizeOriImg, ho_Edges, ho_PolygonsTotal, ho_splitContours;
	HObject ho_SelectedXLD, ho_UnionContours, ho_UnionContoursReConnect, ho_SelectedXLD2;
	HObject ho_RegionSC, ho_RegionSCUnion, ho_RegionSCZoom, ho_RegionSCErosion, ho_RegionOutput;
	HTuple hv_width, hv_height;
	GenEmptyObj(&ho_RegionOutput);
	
	if (a_recipe[RCP_OMIT_PROCESS::WEAKSC_USE].asInt()== true)
	{
			double weak_sc_resize = a_recipe[RCP_OMIT_PROCESS::WEAKSC_RESIZE].asDouble();
			GetImageSize(a_ho_image_input, &hv_width, &hv_height);
			ReduceDomain(a_ho_image_input, a_ho_region_total_roi, &ho_ReducedImg);
			ZoomImageSize(ho_ReducedImg, &ho_ResizeOriImg, hv_width / weak_sc_resize, hv_height / weak_sc_resize, "constant");

			double weak_sc_lower_threshold = a_recipe[RCP_OMIT_PROCESS::WEAKSC_LOWER_TH].asDouble();
			double weak_sc_upper_threshold = a_recipe[RCP_OMIT_PROCESS::WEAKSC_UPPER_TH].asDouble();
			double weak_sc_length_split = a_recipe[RCP_OMIT_PROCESS::WEAKSC_MIN_LENGTH_SPLIT].asDouble();
			EdgesSubPix(ho_ResizeOriImg, &ho_Edges, "canny", 1, weak_sc_lower_threshold, weak_sc_upper_threshold);
			GenPolygonsXld(ho_Edges, &ho_PolygonsTotal, "ramer", 2);
			SplitContoursXld(ho_PolygonsTotal, &ho_splitContours, "polygon", 1, 5);
			SelectShapeXld(ho_splitContours, &ho_SelectedXLD, "contlength", "and", weak_sc_length_split, 999999);

			double weak_sc_min_union_length = a_recipe[RCP_OMIT_PROCESS::WEAKSC_MIN_UNION_LENGTH].asDouble();
			double weak_sc_min_union_max_shift = a_recipe[RCP_OMIT_PROCESS::WEAKSC_MIN_UNION_MAXSHIFT].asDouble();
			double weak_sc_min_union_radian = a_recipe[RCP_OMIT_PROCESS::WEAKSC_MIN_UNION_RADIAN].asDouble();
			UnionCollinearContoursXld(ho_SelectedXLD, &ho_UnionContours, weak_sc_min_union_length, 1, weak_sc_min_union_max_shift, weak_sc_min_union_radian, "attr_forget");
			UnionCollinearContoursXld(ho_UnionContours, &ho_UnionContoursReConnect, 2, 1, 3, 0.785, "attr_forget");
			SelectShapeXld(ho_UnionContoursReConnect, &ho_SelectedXLD2, "contlength", "and", weak_sc_min_union_length, 999999);
			GenRegionContourXld(ho_SelectedXLD2, &ho_RegionSC, "filled");
			Union1(ho_RegionSC, &ho_RegionSCUnion);

			double dilation_value = a_recipe[RCP_OMIT_PROCESS::DILATION].asDouble();
			ZoomRegion(ho_RegionSCUnion, &ho_RegionSCZoom, weak_sc_resize, weak_sc_resize);
			ErosionCircle(ho_RegionSCZoom, &ho_RegionSCErosion, weak_sc_resize / 2 - 0.5); // Resize 한만큼 Region이 커지기 때문에 수축하여 다시 맞춰줌
			DilationCircle(ho_RegionSCErosion, &ho_RegionSCErosion, dilation_value);

			ConcatObj(ho_STDEVRegionOutput, ho_RegionSCErosion, &ho_RegionOutput);

	}
	//--------------------------------------------------------------------------------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------------------------------------------
	// Sticker 영역 찾아서 팽창
	if (a_recipe[RCP_OMIT_PROCESS::STICKER_USE].asInt() == true)
	{

		HObject ho_Omit_STDEV_Copy, ho_blob_region, ho_sticker_region_Closing, ho_sticker_region;

		double ptn_threshold_upper = a_recipe[RCP_OMIT_PROCESS::STDEV_THRESHOLD].asDouble();
		Threshold(a_ho_stdev_image, &ho_Omit_STDEV_Copy, ptn_threshold_upper, 255);
		Connection(ho_Omit_STDEV_Copy, &ho_Omit_STDEV_Copy);
		SelectShape(ho_Omit_STDEV_Copy, &ho_blob_region, "area", "and", a_recipe[RCP_OMIT_PROCESS::STICKER_AREA_MIN].asDouble() / 20, 9999999);
		Union1(ho_blob_region, &ho_blob_region);
		ClosingCircle(ho_blob_region, &ho_sticker_region_Closing, a_recipe[RCP_OMIT_PROCESS::STICKER_CLOSING].asDouble());
		Connection(ho_sticker_region_Closing, &ho_sticker_region_Closing);

		SelectShape(ho_sticker_region_Closing, &ho_sticker_region, "holes_num", "and", a_recipe[RCP_OMIT_PROCESS::STICKER_HOLES_NUM_MIN].asDouble(), 9999999);
		FillUp(ho_sticker_region, &ho_sticker_region);
		SelectShape(ho_sticker_region, &ho_sticker_region, "area", "and", a_recipe[RCP_OMIT_PROCESS::STICKER_AREA_MIN].asDouble(), a_recipe[RCP_OMIT_PROCESS::STICKER_AREA_MAX].asDouble());
		DilationCircle(ho_sticker_region, &ho_sticker_region, a_recipe[RCP_OMIT_PROCESS::STICKER_DILATION].asDouble());

		ConcatObj(ho_RegionOutput, ho_sticker_region, &ho_RegionOutput);
		//--------------------------------------------------------------------------------------------------------------------------------------

		HObject ho_selected_region = GetSelectedRegion(a_recipe, a_ho_image_input, a_ho_region_total_roi);
		ho_RegionOutput = GetEnhanceOmitFilterConcat(ho_RegionOutput, ho_selected_region);
	}

	return ho_RegionOutput;
}

HObject COmitProcess::GetSelectedRegion(Json::Value a_recipe, HObject a_ho_image_input, HObject a_ho_region_total_roi)
{
	HObject ho_selected_region;

	HObject HProcessImage, HExpandedImage, HExpandedImageReduced, HImageMean, HOmitThreshRgn, HConnectedRgn, HSelectedRgn;
	ReduceDomain(a_ho_image_input, a_ho_region_total_roi, &HProcessImage);
	HTuple HDTFilterSizeX, HDTFilterSizeY, HDTValue;
	int nTest = a_recipe[RCP_OMIT_PROCESS::DYNAMIC_FILTER_SIZE].asInt();
	HDTFilterSizeX = HDTFilterSizeY = a_recipe[RCP_OMIT_PROCESS::DYNAMIC_FILTER_SIZE].asInt();
	HDTValue = a_recipe[RCP_OMIT_PROCESS::DYNAMIC_THRESHOLD].asInt();
	ExpandDomainGray(HProcessImage, &HExpandedImage, HDTFilterSizeX / 2);
	ReduceDomain(HExpandedImage, a_ho_region_total_roi, &HExpandedImageReduced);
	MeanImage(HExpandedImageReduced, &HImageMean, HDTFilterSizeX, HDTFilterSizeY);
	DynThreshold(HProcessImage, HImageMean, &HOmitThreshRgn, HDTValue, "light");
	Connection(HOmitThreshRgn, &HConnectedRgn);
	SelectShape(HConnectedRgn, &ho_selected_region, "area", "and", 50, 99999);

	return ho_selected_region;
}

HObject COmitProcess::GetEnhanceOmitFilterConcat(HObject a_ho_region_input, HObject a_ho_selected_region)
{
	HObject ho_RegionOutput;
	ConcatObj(a_ho_region_input, a_ho_selected_region, &ho_RegionOutput);

	return ho_RegionOutput;
}



cv::Mat COmitProcess::GetOmitImage(HObject a_ho_region_total_roi, HObject a_ho_image_input, HObject a_ho_selected_region, double a_resize_factor_vignetting)
{
	HObject HBinImage, HImageAffineTrans, HThresRgn, HConnectedRegions, HSelectedRgn2, HSelectedRgn3, HSelectedRgn4, HMuraOmitRgn;
	HObject HMuraOmitImage;
	
	HTuple HWidth, HHeight, HHomMat2DIdentity, HHomMat2DScale;
	GetImageSize(a_ho_image_input, &HWidth, &HHeight);
	RegionToBin(a_ho_selected_region, &HBinImage, 255, 0, HWidth, HHeight);
	HomMat2dIdentity(&HHomMat2DIdentity);
	double dResizeFactor = 1.0 / a_resize_factor_vignetting;
	dResizeFactor = round(dResizeFactor * 10000) / 10000;
	HomMat2dScale(HHomMat2DIdentity, dResizeFactor, dResizeFactor, 0, 0, &HHomMat2DScale);
	AffineTransImageSize(HBinImage, &HImageAffineTrans, HHomMat2DScale, "constant", HWidth * dResizeFactor, HHeight * dResizeFactor);
	Threshold(HImageAffineTrans, &HThresRgn, 30, 255);
	Connection(HThresRgn, &HConnectedRegions);
	SelectShape(HConnectedRegions, &HSelectedRgn2, "area", "and", 10, 200);
	SelectShape(HSelectedRgn2, &HSelectedRgn3, "ratio", "and", 0, 0.5);
	SelectShape(HSelectedRgn2, &HSelectedRgn4, "ratio", "and", 2, 15);
	Union2(HSelectedRgn3, HSelectedRgn4, &HMuraOmitRgn);
	RegionToBin(HMuraOmitRgn, &HMuraOmitImage, 255, 0, HWidth * dResizeFactor, HHeight * dResizeFactor);

	CHalconMath halcon;

	// SelectShape에서 Omit 후보가 모두 Filtering 되어 ValidRegion이 하나라도 false가 나온다면 빈 이미지를 내뱉는다.
	if (halcon.ValidHRegion(HMuraOmitRgn) == false)
		return cv::Mat((int)HHeight, (int)HWidth, CV_8U, cv::Scalar::all(0));

	HTuple hlImageWidth, hlImageHeight;
	cv::Mat matOmitImage = halcon.HimageToCvmat(HMuraOmitImage, hlImageWidth, hlImageHeight);

	return matOmitImage;
}

HObject COmitProcess::GetSharedSelectedRegion(Json::Value a_recipe, HObject a_ho_image_input, HObject a_ho_region_total_roi)
{
	HObject ho_shared_selected_region, ho_SelectedRegionsOri;

	Intersection(a_ho_image_input, a_ho_region_total_roi, &a_ho_image_input);
	//이진화 된 region들 Blob
	Connection(a_ho_image_input, &ho_SelectedRegionsOri);
	double bolb_area_size = a_recipe[RCP_OMIT_PROCESS::SHARED_BLOB_MIN_AREA].asDouble();
	SelectShape(ho_SelectedRegionsOri, &ho_shared_selected_region, "area", "and", bolb_area_size, 9999999); //원본 5이고 Recipe로 뺏음
	
	Intersection(ho_shared_selected_region, a_ho_region_total_roi, &ho_shared_selected_region);
	Union1(ho_shared_selected_region, &ho_shared_selected_region);
	Connection(ho_shared_selected_region, &ho_shared_selected_region);

	return ho_shared_selected_region;
}

HObject COmitProcess::GetInspectionRoi(Json::Value a_recipe, HObject a_ho_src_image)
{
	HObject ho_result;
	HObject ho_Region, ho_ConnectedRegions, ho_SelectedRegions, ho_RegionFillUp;
	HTuple hv_Index, hv_Row1, hv_Column1, hv_Row2, hv_Column2;
	HTuple hv_Avg_Pre, hv_dSd_Pre;

	// Omit PTN
	Threshold(a_ho_src_image, &ho_Region, a_recipe[RCP_OMIT_PROCESS::INSP_ROI_MIN_THRESHOLD].asDouble(), a_recipe[RCP_OMIT_PROCESS::INSP_ROI_MAX_THRESHOLD].asDouble());
	OpeningCircle(ho_Region, &ho_ConnectedRegions, 1.5);
	ClosingCircle(ho_ConnectedRegions, &ho_ConnectedRegions, 3.5);
	Connection(ho_ConnectedRegions, &ho_ConnectedRegions);
	SelectShapeStd(ho_ConnectedRegions, &ho_SelectedRegions, "max_area", 70);
	FillUp(ho_SelectedRegions, &ho_RegionFillUp);

	// 검사 영역 Region 저장
	SmallestRectangle1(/*ho_SelectedRegions*/ho_RegionFillUp, &hv_Row1, &hv_Column1, &hv_Row2, &hv_Column2);

	GenRectangle1(&ho_result, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
	ErosionCircle(ho_result, &ho_result, 1.5);

	Intensity(ho_result, a_ho_src_image, &hv_Avg_Pre, &hv_dSd_Pre);

	return ho_result;
}


void COmitProcess::CalcOmitDefectData(HObject a_omit_Insp_roi,HObject a_ho_result_roi, int &a_omit_result_area_count ,int &a_omit_result_area_rate)
{
	HTuple hv_Insp_Area, hv_Insp_ResultArea;

	AreaCenter(a_omit_Insp_roi, &hv_Insp_Area,NULL,NULL);
	AreaCenter(a_ho_result_roi,&hv_Insp_ResultArea,NULL,NULL);

	a_omit_result_area_count = (int)(double)hv_Insp_ResultArea;
	int dddd = (int)(double)hv_Insp_Area;
	a_omit_result_area_rate = (int)((double)hv_Insp_ResultArea / (double)(hv_Insp_Area) * 10000.0);
}

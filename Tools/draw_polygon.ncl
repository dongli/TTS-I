load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"
    projection@angle = 0.5
    line_width = 1.0
    point_size = 0.01

    if (.not. isdefined("file_name")) then
        system("echo Input the polygon file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    Rad2Deg = 45.0/atan(1.0)

    content = asciiread(file_name, -1, "double")
    num_vertex = dimsizes(content)/4
    ; vertices
    vertex_lon = new(num_vertex+1, double)
    vertex_lat = new(num_vertex+1, double)
    vertex_lon(0:num_vertex-1) = content(0:dimsizes(content)/2-1:2)*Rad2Deg
    vertex_lat(0:num_vertex-1) = content(1:dimsizes(content)/2-1:2)*Rad2Deg
    vertex_lon(num_vertex) = vertex_lon(0)
    vertex_lat(num_vertex) = vertex_lat(0)
    ; test points
    testpoint_lon = new(num_vertex+1, double)
    testpoint_lat = new(num_vertex+1, double)
    testpoint_lon(0:num_vertex-1) = \
        content(dimsizes(content)/2:dimsizes(content)-1:2)*Rad2Deg
    testpoint_lat(0:num_vertex-1) = \
        content(dimsizes(content)/2+1:dimsizes(content)-1:2)*Rad2Deg
    testpoint_lon(num_vertex) = testpoint_lon(0)
    testpoint_lat(num_vertex) = testpoint_lat(0)

    allpoint_lon = new(2*num_vertex+1, double)
    allpoint_lat = new(2*num_vertex+1, double)

    allpoint_lon(0:2*num_vertex-1:2) = vertex_lon(0:num_vertex-1)
    allpoint_lon(1:2*num_vertex-1:2) = testpoint_lon(0:num_vertex-1)
    allpoint_lat(0:2*num_vertex-1:2) = vertex_lat(0:num_vertex-1)
    allpoint_lat(1:2*num_vertex-1:2) = testpoint_lat(0:num_vertex-1)
    allpoint_lon(2*num_vertex) = allpoint_lon(0)
    allpoint_lat(2*num_vertex) = allpoint_lat(0)

    wks = gsn_open_wks("pdf", file_name)

    res_map = setup_map(projection)
    res_map@mpCenterLonF = calc_polygon_center_lon(vertex_lon)
    res_map@mpCenterLatF = avg(vertex_lat)
    map = gsn_csm_map(wks, res_map)

    res_line                        = True
    res_line@gsLineThicknessF       = line_width
    res_point                       = True
    res_point@gsMarkerSizeF         = point_size

    res_line@gsLineColor            = "blue"
    gsn_polyline(wks, map, vertex_lon, vertex_lat, res_line)

    res_point@gsMarkerIndex         = 1
    gsn_polymarker(wks, map, vertex_lon, vertex_lat, res_point)

    res_line@gsLineColor            = "red"
    res_line@gsLineDashPattern      = 2
    gsn_polyline(wks, map, allpoint_lon, allpoint_lat, res_line)

    res_point@gsMarkerColor         = "red"
    gsn_polymarker(wks, map, testpoint_lon, testpoint_lat, res_point)

    frame(wks)

end

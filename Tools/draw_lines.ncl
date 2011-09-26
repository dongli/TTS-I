load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"

    if (.not. isdefined("file_name")) then
        system("echo Input the node coordinates file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    if (projection .eq. "ST") then
        if (.not. isdefined("angle")) then
            system("echo Input view angle:")
            angle = stringtofloat(systemfunc("read -p ' > ' ans; echo $ans"))
        end if
        projection@angle = angle
    end if

    contents = asciiread(file_name, -1, "double")

    num_point = floattoint(dimsizes(contents)/2.0)
    point_lon = new(num_point, double)
    point_lat = new(num_point, double)

    point_lon = contents(0:dimsizes(contents)-1:2)
    point_lat = contents(1:dimsizes(contents)-1:2)

    Rad2Deg = 45.0/atan(1.0)
    point_lon = point_lon*Rad2Deg
    point_lat = point_lat*Rad2Deg

    res_map = setup_map(projection)

    res_line                        = True
    res_line@gsLineThicknessF       = 0.5
    res_line@gsLineColor            = "blue"

    res_point                       = True
    res_point@gsMarkerIndex         = 1
    res_point@gsMarkerSizeF         = 0.01

    wks = gsn_open_wks("newpdf", file_name)

    if (projection .eq. "ST") then
        res_map@mpCenterLonF = calc_polygon_center_lon(point_lon)
        res_map@mpCenterLatF = avg(point_lat)
    end if
    map = gsn_csm_map(wks, res_map)

    gsn_polyline(wks, map, point_lon, point_lat, res_line)

    gsn_polymarker(wks, map, point_lon, point_lat, res_point)

    frame(wks)

end

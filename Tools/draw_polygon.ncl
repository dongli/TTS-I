load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"
    projection@angle = 1.0
    projection@min_lat = 80.0
    projection@max_lat = -80.0
    line_width = 1.0
    point_size = 0.01

    if (.not. isdefined("file_name")) then
        system("echo Input the polygon file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    content = readAsciiTable(file_name, 3, "double", 0)
    
    dims =  dimsizes(content)

    ; --------------------------------------------------------------------------
    ; get polygon vertices
    num_vertex = 0
    do i = 0, dims(0)-1
        if (content(i,0) .eq. 0) then
            num_vertex = num_vertex+1
        end if
    end do
    vertex_lon = new(num_vertex+1, double)
    vertex_lat = new(num_vertex+1, double)
    j = 0
    do i = 0, dims(0)-1
        if (content(i,0) .eq. 0) then
            vertex_lon(j) = content(i,1)*Rad2Deg
            vertex_lat(j) = content(i,2)*Rad2Deg
            j = j+1
        end if
    end do
    vertex_lon(num_vertex) = vertex_lon(0)
    vertex_lat(num_vertex) = vertex_lat(0)

    print(vertex_lon)
    print(vertex_lat)

    ; --------------------------------------------------------------------------
    ; get test points
    num_testpoint = dims(0)-num_vertex
    if (num_testpoint .ne. 0) then
        testpoint_lon = new(num_testpoint, double)
        testpoint_lat = new(num_testpoint, double)
        j = 0
        do i = 0, dims(0)-1
            if (content(i,0) .eq. 1) then
                testpoint_lon(j) = content(i,1)*Rad2Deg
                testpoint_lat(j) = content(i,2)*Rad2Deg
                j = j+1
            end if
        end do

        allpoint_lon = new(2*num_vertex+1, double)
        allpoint_lat = new(2*num_vertex+1, double)
    
        allpoint_lon(0:2*num_vertex-1:2) = vertex_lon(0:num_vertex-1)
        allpoint_lon(1:2*num_vertex-1:2) = testpoint_lon(0:num_vertex-1)
        allpoint_lat(0:2*num_vertex-1:2) = vertex_lat(0:num_vertex-1)
        allpoint_lat(1:2*num_vertex-1:2) = testpoint_lat(0:num_vertex-1)
        allpoint_lon(2*num_vertex) = allpoint_lon(0)
        allpoint_lat(2*num_vertex) = allpoint_lat(0)
    end if

    ; --------------------------------------------------------------------------
    wks = gsn_open_wks("pdf", file_name)

    projection@center_lon = new(1, double)
    projection@center_lat = new(1, double)
    calc_polygon_center(vertex_lon, vertex_lat, \
                        projection@center_lon, projection@center_lat)
    res_map = True
    setup_map(projection, res_map)
    map = gsn_csm_map(wks, res_map)

    draw(map)

    ; --------------------------------------------------------------------------
    res_line                        = True
    res_line@gsLineThicknessF       = line_width
    res_point                       = True
    res_point@gsMarkerSizeF         = point_size

    res_line@gsLineColor            = "black"
    gsn_polyline(wks, map, vertex_lon, vertex_lat, res_line)

    res_point@gsMarkerIndex         = 1
    gsn_polymarker(wks, map, vertex_lon, vertex_lat, res_point)

    if (num_testpoint .ne. 0) then
        res_line@gsLineColor        = "red"
        res_line@gsLineDashPattern  = 2
        gsn_polyline(wks, map, allpoint_lon, allpoint_lat, res_line)

        res_point@gsMarkerColor     = "red"
        gsn_polymarker(wks, map, testpoint_lon, testpoint_lat, res_point)
    end if

    ; draw poles as references
    res_point@gsMarkerColor = "green"
    gsn_polymarker(wks, map, (/0.0,0.0/), (/90.0,-90.0/), res_point)

    frame(wks)

end

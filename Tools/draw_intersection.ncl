load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"
    projection@angle = 4.0
    projection@min_lat = 85.0
    ;projection@max_lat = -55.0
    ;projection@center_lon = 0.0
    line_width = 3.0
    point_size = 0.05

    if (.not. isdefined("file_name")) then
        system("echo Input the polygon with intersections file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    content = readAsciiTable(file_name, 3, "double", 0)

    dims = dimsizes(content)

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
    ; get intersection
    num_intersect = dims(0)-num_vertex
    if (num_intersect .ne. 0) then
        intersect_lon = new(num_intersect, double)
        intersect_lat = new(num_intersect, double)
        j = 0
        do i = 0, dims(0)-1
            if (content(i,0) .eq. 1) then
                intersect_lon(j) = content(i,1)*Rad2Deg
                intersect_lat(j) = content(i,2)*Rad2Deg
                j = j+1
            end if
        end do
    end if

    ; --------------------------------------------------------------------------
    wks = gsn_open_wks("newpdf", file_name)

    res_map = True
    if (projection .eq. "ST") then
        projection@center_lon = calc_polygon_center_lon(vertex_lon)
        projection@center_lat = avg(vertex_lat)
    end if
    setup_map(projection, res_map)
    map = gsn_csm_map(wks, res_map)

    draw(map)

    ; --------------------------------------------------------------------------
    ; draw point counter mesh
    f = addfile("point_counter_bound_mesh.nc", "r")
    if (projection .eq. "ST") then
        min_lon = res_map@mpCenterLonF-10.0
        max_lon = res_map@mpCenterLonF+10.0
        min_lat = res_map@mpCenterLatF-10.0
        max_lat = res_map@mpCenterLatF+10.0
    else
        min_lon = 0.0d0
        max_lon = 360.0d0
        min_lat = -90.0d0
        max_lat = 90.0d0
        if (projection .eq. "CE") then
            if (isatt(projection, "min_lat")) then
                min_lat = projection@min_lat
            end if
            if (isatt(projection, "max_lat")) then
                max_lat = projection@max_lat
            end if
        end if
        if (projection .eq. "NH") then
            min_lat = projection@min_lat
        end if
        if (projection .eq. "SH") then
            max_lat = projection@max_lat
        end if
    end if
    plot_lon_lat_mesh(wks, map, f->lon, f->lat, \
        min_lon, max_lon, min_lat, max_lat)

    res_line                        = True
    res_line@gsLineThicknessF       = line_width
    res_line@mpGreatCircleLinesOn   = True
    res_point                       = True
    res_point@gsMarkerSizeF         = point_size

    res_line@gsLineColor            = "blue"
    gsn_polyline(wks, map, vertex_lon, vertex_lat, res_line)

    res_point@gsMarkerIndex         = 1
    gsn_polymarker(wks, map, vertex_lon, vertex_lat, res_point)

    if (num_intersect .ne. 0) then
        res_point@gsMarkerColor     = "red"
        gsn_polymarker(wks, map, intersect_lon, intersect_lat, res_point)
    end if

    frame(wks)

end

var mymap = L.map('mapid').setView([25.149681684332624 ,121.7768906056881], 20);
L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    maxZoom: 30,
    //id: 'mapbox/light-v9',
    id: 'mapbox/streets-v10', //8 9 10   
    //id: 'mapbox/satellite-v9', // 8 9    
    accessToken: 'pk.eyJ1Ijoic3RldmVsaWFvMTY4OCIsImEiOiJjazE3OGYyeDkxY3hsM250amdvdjBqMGFvIn0.Lv9UAvyh3NYLGGz1WGy3Ag' 
}).addTo(mymap);

//--------------- Show onclick position in map -------------------
var popup = L.popup();
function onMapClick(e) {
    popup
        .setLatLng(e.latlng)
        .setContent("You clicked the map at " + e.latlng.toString())
        .openOn(mymap);
}
mymap.on('click', onMapClick);
//-------------------------------------------------------------
mapMarkers1 = [];
mapMarkers2 = [];
mapMarkers3 = [];
//path = [];

var source = new EventSource('eventsource/'); //ENTER YOUR TOPICNAME HERE

source.addEventListener('message', function(e){
  console.log('Message');
  obj = JSON.parse(e.data);
  console.log(obj);
  
  if(obj.channel == '00001') {   
    //Mark the 'trash','cap', 'plastic_bag'if it is identified by yolov3 
    trash_marker = L.circleMarker([obj.latitude, obj.longitude], {
      color: 'green',
      fillColor: '#f03',
      fillOpacity: 0.5,
      radius: 2 }).addTo(mymap); 
    trash_marker.bindTooltip("lon:"+obj.longitude
      +"<br>lat:"+obj.latitude+"<br>trash:"+obj.trash_num+"<br>cap:"+obj.cap_num+"<br>plastic_bag:"+obj.plastic_bag_num+"<br>time:"+obj.timestamp).openTooltip();
    mapMarkers1.push(trash_marker);    
    //trash_marker.bindTooltip("lon:"+obj.longitude).openTooltip();    
  }

  if(obj.channel == '00002') {
    //mark the current copter position
    for (var i = 0; i < mapMarkers2.length; i++) {
      mymap.removeLayer(mapMarkers2[i]);
    }
    marker = L.marker([obj.latitude, obj.longitude]).addTo(mymap);
    //marker.bindPopup("lon:"+obj.longitude+"<br>lat:"+obj.latitude).openPopup();    
    mapMarkers2.push(marker);    
  }

  if(obj.channel == '00003') {
    //mark the path on map
    //num_wp = obj.num_wp    
    circle = L.circleMarker([obj.latitude, obj.longitude], {
      color: 'red',
      fillColor: '#f03',
      fillOpacity: 0.5,
      radius: 5 }).addTo(mymap);   
      mapMarkers3.push([obj.latitude, obj.longitude]);    
    /*
    if(obj.path_is_ok==1){//plot path
      polyline =  L.polyline(path, {color: 'pink',weight:5,opacity:0.3,}).addTo(mymap);
    }*/
    circle.bindTooltip("WP"+ obj.waypoint).openTooltip();
    /* if(mapMarkers3.length==num_wp){
      for (var i = 0; i < mapMarkers3.length; i++) {
        mymap.removeLayer(mapMarkers3[i]);
      }
    }  */
  }

  if(obj.channel == '00004') {
    //mark the HOME   
    home = L.circleMarker([obj.latitude, obj.longitude], {
      color: 'green',
      fillColor: '#f03',
      fillOpacity: 0.4,
      radius: 10 }).addTo(mymap);   
       
    home.bindTooltip("HOME").openTooltip();
  }

  if(obj.channel == '00009') {                  
    document.getElementById("drone_status").innerHTML =
          "<b> [    Lat , Lon   ] : " + obj.latitude      + "  , "      + obj.longitude+"<b>"
        + "<br>[      Alt       ] : " + obj.altitude      + " <b>(m)"
        + "<br>[   Ground speed ] : " + obj.groundspeed   + "<b>(m/sec)"
        + "<br>[   Heading      ] : " + obj.heading       + "<b>(deg)"
        + "<br>[      Mode      ] : " + obj.mode 
        + "<br>[ Velocity_North ] : " + obj.velocity_N    + "<b>(m/sec)"
        + "<br>[ Velocity_East  ] : " + obj.velocity_E    + "<b>(m/sec)"
        + "<br>[ Velocity_Down  ] : " + obj.velocity_Down + "<b>(m/sec)"
        + "<br>[      Yaw       ] : " + obj.yaw           + "<b>(deg)"
        + "<br>[      Roll      ] : " + obj.roll          + "<b>(deg)"
        + "<br>[      Pitch     ] : " + obj.pitch         + "<b>(deg)"            
        + "<br>[  Dist to home  ] : " + obj.dist_to_home  + "<b>(m)"
        + "<br>[  Dist to Target  ] : " + obj.dist_to_target  + "<b>(m)"
        + "<br>[   Gps   ] : " + obj.gps_status+"<b>"
        + "<br>[   Batt   ] : " + obj.battery+"<b>"
  } 
}, false);

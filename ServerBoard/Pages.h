/* ToDo:
[X] List of WiFi Addreses
[ ] The actual sites
>  [ ] Paint Js
>  [X] CSS
>  [X] HTML
[ ] comms site <=> uC
[ ] The controller
[ ] Auto tunning maybe
*/

// I won't comment the HTML code, so forget 'bout it

// site variables
String openLoop = "<html><h1>this shi aint workin</h1></html>";
String lobby = "<html><h1>this shi aint workin</h1></html>";
String notebook = "<html><h1>this shi aint workin</h1></html>";
String canvas = "<html><h1>this shi aint workin</h1></html>";

void BuildSites (){
// Building the sites

  // Site designed to communicate to the uC for an Open Loop Controller
  openLoop = "<html><body><center>";
  openLoop += "<head><meta name=\"viewport\" content=\"width=device-width\">";
  openLoop += "<title>IoT Cartesiano</title>";
  openLoop += "<h1>Interfaz IoT</h1>";
  openLoop += "Controles del cartesiano";
  openLoop += "<form method='get'>";
  openLoop +=   "<button type='submit' name='Xaxis' value='increment'>X+</button>";
  openLoop +=   "<button type='submit' name='Xaxis' value='decrement'>X-</button>";
  openLoop +=   "<button type='submit' name='Xaxis' value='Stop'>Stop</button>";
  openLoop += "</form>";
  openLoop += "<form method='get'>";
  openLoop +=   "<button type='submit' name='Yaxis' value='increment'>Y+</button>";
  openLoop +=   "<button type='submit' name='Yaxis' value='decrement'>Y-</button>";
  openLoop +=   "<button type='submit' name='Yaxis' value='Stop'>Stop</button>";
  openLoop += "</form>";
  openLoop += "</center></body></html>";

  // Lobby for selecting which page to enter for the closed loop controller, the sensor or the SP sender
  lobby =  "<html> <head> <meta charset='utf-8'/>";
	lobby +=  	"<link rel='stylesheet' type='text/css' href='https://raw.githack.com/cyberlong/web-file-hosting/main/dastyle.css'>";
	lobby +=  	"<title>Bonjour</title> </head> <body>";
	lobby +=  	"<h1 class='title'>IoT Cartesiano</h1>";
	lobby +=  	"<p id='context'>Plataforma IoT del plano cartesiano. Esta pagina es servida por la maquina y los botones redireccionan a los sitios respectivos</p> <div id='button-container'>";
	lobby +=  		"<div id='canvas-link' class='ui-button'>";
	lobby +=  			"<img class='icon' src='https://www.svgrepo.com/show/160785/canvas.svg'>";
	lobby +=  			"<a href='./canvas/'><button class='button'>Canvas</button></a>";
	lobby +=  		"</div> <div id='notebook-link' class='ui-button'>";
	lobby +=  			"<img class='icon' src='https://www.svgrepo.com/show/1693/notebook.svg'>";
	lobby +=  			"<a href='./notebook/'><button class='button'>Notebook</button></a>";
	lobby +=  		"</div> <div id='calibrate-link' class='ui-button'>";
	lobby +=  			"<img class='icon' src='https://www.svgrepo.com/show/414274/calibrate.svg'>";
	lobby +=  			"<a href='./'><button class='button'>Calibrate</button></a>";
	lobby +=  		"</div> </div> <div class='bottom-container'>";
	lobby +=  		"<img class='bottom-img' src='https://i1.pickpik.com/photos/198/292/877/notepad-pencil-pen-paper-136f37277e02773426dada44542d9f2a.jpg'/>";
	lobby +=  	"</div> </body> </html>";
  // The Canvas is responsable for storing in cache and sending the setpoints to the uC
  canvas =  "<html> <head> <meta charset='utf-8'/>";
	canvas +=  	"<title>Well hello there</title>";
	canvas +=  	"<link rel='stylesheet' type='text/css' href='https://raw.githack.com/cyberlong/web-file-hosting/main/dastyle.css'>";
	canvas +=  "</head> <body> <h1 class='title'>Canvas</h1>";
	canvas +=  	"<p id='context'>Dibuja en el siguiente recuadro el camino que se desea";
	canvas +=  		"que la maquina siga, luego presione \"Hecho\" para mandar la instrucción.";
	canvas +=  	"</p> <div class='bottom-container'>";
	canvas +=  		"<img src='https://i.pinimg.com/originals/eb/ce/e7/ebcee7e05c657721afdaee9ff6141e70.png'>";
	canvas +=  	"</div> <div id='button-container'> <div id='back' class='ui-button'>";
	canvas +=  			"<img class='icon' src='https://www.svgrepo.com/show/500472/back.svg'>";
	canvas +=  			"<a href='../'><button class='button'>Atras</button></a>";
	canvas +=  		"</div> <div id='send-instructions' class='ui-button'>";
	canvas +=  			"<img class='icon' src='https://www.svgrepo.com/show/504730/pencil.svg'>";
	canvas +=  			"<a href='./'><button class='button'>Hecho</button></a>";
	canvas +=  		"</div> </div> </body> </html>";

  // The sensor page is placed on the bed, the pencil touches the screen and its location is sent to the uC
  notebook =  "<html> <head> <meta charset='utf-8'/>";
	notebook +=  	"<title>Why is it 4am?</title>";
	notebook +=  	"<link rel='stylesheet' type='text/css' href='https://raw.githack.com/cyberlong/web-file-hosting/main/dastyle.css'>";
	notebook +=  "</head> <body> <h1 class='title'>Notebook</h1> <p id='context'>";
	notebook +=  		"Esto es el cuaderno de la maquina. Esta debijará en esta pagina siguiendo";
	notebook +=  		"el camino que se le haya marcado en el Canvas.";
	notebook +=  	"</p> <div class='bottom-container'>";
	notebook +=  		"<img src='https://i.pinimg.com/originals/eb/ce/e7/ebcee7e05c657721afdaee9ff6141e70.png'> </div> </body> </html>";

}


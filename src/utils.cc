#include <utils.hh>

namespace utils{
	
	/*void elevationDataToVector(std::string fileName, std::map<int32_t, std::tuple<double, double,int32_t> >& elevData)
	{
		elevData.clear();
		std::string separador = ":";
	
		std::ifstream elevFile;
		elevFile.open(fileName);
		if(!elevFile.good()){
			utilsException mException;
			mException.msg= "Error en acceso a archivo: " + fileName;
			throw mException;
		}
	
		std::string line;
	    while (std::getline(elevFile, line)) {
			std::string field;
		
			std::vector<std::string> buffer;
		    std::istringstream lineStream;
		    lineStream.str(line);

			while (std::getline(lineStream, field, ':')) {
		        buffer.push_back(field);

		    }
		
			if(buffer[3] == "" || buffer.size() == 3){
				buffer[3] = "-1";
			}
			
			int32_t idPatchAgent = std::stoi(buffer[0]);
			double  lat          = std::stod(buffer[1]);
			double  lon          = std::stod(buffer[2]);
			int32_t elevation    = std::stoi(buffer[3]);
		
			elevData[idPatchAgent] = std::make_tuple(lat, lon, elevation);
	    }
	    elevFile.close();
	
	}*/
	
	utils::uuidSimulation_t get_uuid()
    {
        std::string res;

        uuid_t uuid;
        char uuid_cstr[37]; // 36 byte uuid plus null.
        uuid_generate(uuid);
        uuid_unparse(uuid, uuid_cstr);
        res = std::string(uuid_cstr);


        return res;
    } 
	
	//
	// Devuelve el contenido JSON del servicio apuntado por urlRest
	void restClient_get(const std::string restURL, json& response)
	{
		RestClient::Response r;
		
		r = RestClient::get(restURL );

		switch(r.code){
			case COULDNT_CONNECT:{
				utilsException uException;
				uException.msg= "Could not connect to REST server.";
				throw uException;
				break;
			}
			case 200:{
				response = json::parse(r.body);
	
				break;
			}
			default:{
				response["responseCode"] = r.code;
				break;
			}
		}
	}
	
	/*
	void transformData(json response, std::string KEY, std::vector<utils::timeSerie_t>& outTS)
	{
		outTS.clear();
		json dataToProc;

		dataToProc = response["data"];

		uint32_t totalItems = response["data"].size();

		for(size_t idx = 0; idx < totalItems; idx++){
			json tsJSON;
			
			
			// Del registro en curso, se saca el UUID
			//std::string registerUUID;
			//try{
			//	registerUUID = dataToProc[idx]["uuid"].get<std::string>();
			//}
			//catch(std::exception& e){
			//	registerUUID = "--NULL--";
			//}
			//std::cout << "\nUUID=" << registerUUID << std::endl;
			
			
			// Se almacena la serie de tiempo de interes.
			// tiene la forma "tiempo": valor.
			// El tiempo está como string en el JSON.
			tsJSON = dataToProc[idx][KEY];
	
			// Se crea la serie de tiempo a partir de los datos JSON
			utils::timeSerie_t ts;
			for(const auto& it : tsJSON.items() ){
				uint32_t timeStamp = std::stoi(it.key());
				//std::cout << it.key() << ":" << it.value() << std::endl;
				double value = it.value();
				ts[timeStamp] = value;
			}
	
			//Finalmente, la serie de tiempo creada se agrega a la lista de 
			//series de tiempo de la ciudad consultada.
			outTS.push_back(ts);
		}
	}
	*/
	
	/*void transformRawDataJsonTOtsGlobal(json dataRaw, std::string KEY, utils::timeSerie_t& outTS)
	{
		 json evacByZone;
	 
		 outTS.clear();
		 evacByZone = dataRaw[KEY];
		 for(const auto& tStamp : evacByZone.items()){
		 	 uint32_t timeStamp = std::stoi(tStamp.key());
			 json zonesData = tStamp.value();
		 
			 double evacPerc = 0.0;
			 for(const auto& zone : zonesData.items()){
			 	json timeEvac = zone.value();
				evacPerc += timeEvac[1].get<double>();
			 }
		 
			 outTS[timeStamp] = evacPerc;
	  
		 }
	}*/
	
	void transformRawDataJsonTOtsGlobal(json dataRaw, std::string KEY, json& outJSON)
	{
		 json evacByZone;
		 utils::timeSerie_t outTS;
			 
		 evacByZone = dataRaw[KEY];
		 for(const auto& tStamp : evacByZone.items()){
		 	 uint32_t timeStamp = std::stoi(tStamp.key());
			 json zonesData = tStamp.value();
		 
			 double evacPerc = 0.0;
			 for(const auto& zone : zonesData.items()){
			 	json timeEvac = zone.value();
				evacPerc += timeEvac[1].get<double>();
			 }
		 
			 outTS[timeStamp] = evacPerc;
	  
		 }
		 
 		//utils::timeSerie_t listEvacAgents;
 		//utils::transformRawDataJsonTOtsGlobal(dataIN, utils::key::evacByZone, listEvacAgents);		
 		for(const auto& [t,v]: outTS){
 			//ts["evacAll"].emplace(std::to_string(t),v);
			outJSON.emplace(std::to_string(t),v);
 		}
	}
	

	void transformRawDataJsonTOtsLocalByZone(json dataRaw, json& outJSON)
	{		
		json evacByZone, zonesInfo;
		evacByZone = dataRaw[utils::key::evacByZone];
		zonesInfo  = dataRaw[utils::key::zonesInfo];
		
		// La clave de timeSerie_json_t es del tipo std::string.
		// Esto es útil para realizar la conversión a JSON en un sólo paso.
		std::map<std::string, std::map<std::string, timeSerie_json_t> > zonesTS;
		
		for(const auto& item : evacByZone.items()){
			std::string timeStamp = item.key();
			
			json zonesData = item.value();
			for(const auto& zone : zonesData.items()){
				std::string zoneID;
				json zoneData;
				
				zoneID   = zone.key();    //"Z1", "Z2", ... , "Zn"
				zoneData = zone.value();  //<#evac, %evac, tevac_min, tevac_max, tevac_mean>
					
				zonesTS[zoneID]["evacData"][timeStamp] = zoneData[1].get<double>();				
			}
		}		
		// convertir el map zoneTS a JSON
		outJSON = json(zonesTS);
		
		// Se agrega los datos geográficos (centroide, requiv) a cada zona
		for(const auto& zone : zonesInfo["reference_zones"].items()){
			std::string zoneID;
			zoneID = zone.key();

			outJSON[zoneID]["geographicData"] = zone.value();
			
		}
		
	}
	
	void extractTimeValueFromTS(const utils::timeSerie_t& ts, utils::timestampSerie_t& timeStamp, utils::dataSerie_t& value, const uint32_t cutIn)
	{
		timeStamp.clear();
		value.clear();
		uint32_t count = 0;
		for(const auto& [tStamp, val] : ts){
			if(cutIn > 0 && count >= cutIn) { break; }
			timeStamp.push_back(tStamp);
			value.push_back(val);
			count++;
		
		}

	}
	
	void cutTimeSerie(const utils::timeSerie_t& tsIn, const double& cutValue, utils::timeSerie_t& tsOut)
	{
		tsOut.clear();
		
		for(const auto& [tStamp, value] : tsIn){
			if(value <= cutValue){
				tsOut[tStamp] = value;
			}
			else{
				// Esto se puede hacer debido a que la ts es un 
				// map (los map son ordenados por la clave)
				break;
			}
		}
		
		
	}
	
	/*
	void fetchTSdata(std::string queryURL, std::vector<utils::timeSerie_t>& outTS)
	{
		RestClient::Response r;
		json response;

		outTS.clear();
		r = RestClient::get(queryURL);
		
		switch(r.code){
			case COULDNT_CONNECT:{
				std::cerr << "Could not connect to REST server." << std::endl;
			}
			case 200:{
				response = json::parse(r.body);
				
				transformData(response, utils::key::evacByZoneAll, outTS); //"evacByZone", "nonEvacByAge"
			
			
				
				//for(const auto& ts : listTS){
				//	std::cout << "----------" << std::endl;
				//	for( const auto& [t,v] : ts ){
				//		std::cout << t << ":" << v << std::endl;
				//	}
				//}
				
			
				break;
			}
			default:{
				std::cerr << "Code:" << r.code << std::endl;
			}
		}

	}
	*/
	
	///////////////////////
	// Caso en que los JSON se almacenan en std::map para diferenciar
	// las distintas simulaciones a traves de su identificador uuid
	void transformData(json response, std::string KEY, utils::listTS_t& outTS)
	{
		outTS.clear();
		json dataToProc;

		
		try{
			dataToProc = response["data"];
		}
		catch(std::exception& e){
			outTS.clear();
			return;
		}

		uint32_t totalItems = response["data"].size();

		for(size_t idx = 0; idx < totalItems; idx++){
			json tsJSON;
			
			
			// Del registro en curso, se saca el UUID
			utils::uuidSimulation_t uuidSimulation;
			try{
				uuidSimulation = dataToProc[idx]["uuid"].get<std::string>();
			}
			catch(std::exception& e){
				uuidSimulation = "--NULL--";
			}
			//std::cout << "\uuidSimulation=" << uuidSimulation << std::endl;
			
			// Del registro en curso, se saca la descripcióm
			std::string descriptionSimulation;
			descriptionSimulation = dataToProc[idx]["description"].get<std::string>();
			
			// Se almacena la serie de tiempo de interes.
			// tiene la forma "tiempo": valor.
			// El tiempo está como string en el JSON.
			tsJSON = dataToProc[idx][KEY];
	
			// Se crea la serie de tiempo global de evacuación a partir de los datos JSON
			utils::timeSerie_t ts;
			for(const auto& it : tsJSON.items() ){
				uint32_t timeStamp = std::stoi(it.key());
				//std::cout << it.key() << ":" << it.value() << std::endl;
				double value = it.value();
				ts[timeStamp] = value;
			}
	
			//Finalmente, la serie de tiempo creada se agrega a la lista de 
			//series de tiempo de la ciudad consultada.
			//outTS[uuidSimulation] = std::make_tuple(descriptionSimulation, ts);
			outTS[uuidSimulation] = std::make_tuple(descriptionSimulation, ts, dataToProc[idx]["evacByZone"]); //2022-07-18
		}
	}
	/*
	void fetchTSdata(std::string queryURL, std::map<utils::uuidSimulation_t, std::tuple<std::string, utils::timeSerie_t> >& outTS, bool clearOut)
	{
		RestClient::Response r;
		json response;

		if(clearOut){
			outTS.clear();
		}
		
		r = RestClient::get(queryURL);
		
		switch(r.code){
			case COULDNT_CONNECT:{
				utilsException uException;
				uException.msg= "Could not connect to REST server.";
				throw uException;
				//std::cerr << "Could not connect to REST server." << std::endl;
				break;
			}
			case 200:{
				response = json::parse(r.body);		
				transformData(response, utils::key::evacByZoneAll, outTS); 
	
				
				//for(const auto& ts : listTS){
				//	std::cout << "----------" << std::endl;
				//	for( const auto& [t,v] : ts ){std::cout << t << ":" << v << std::endl;}
				//}
				
			
				break;
			}
			default:{
				std::cerr << "Code:" << r.code << std::endl;
				break;
			}
		}
	}*/
	
	void fetchSimData(const std::string& queryURL, const utils::uuidSimulation_t& uuidSim, json& response)
	{
		RestClient::Response r;

		
		r = RestClient::get(queryURL+uuidSim);
		
		switch(r.code){
			case COULDNT_CONNECT:{
				utilsException uException;
				uException.msg= "Could not connect to REST server.";
				throw uException;
				
				//std::cerr << "Could not connect to REST server." << std::endl;
				break;
			}
			case 200:{
				response = json::parse(r.body);
				
				//std::cout << response << std::endl;
				
				//transformData(response, utils::key::evacByZoneAll, outTS); //"evacByZone", "nonEvacByAge"
			
			
				break;
			}
			default:{
				std::cerr << "Code:" << r.code << std::endl;
				break;
			}
		}
	}
	
	//
	// transforma una serie de tiempo timeSerie_t (AKA std::map<uint32_t, double>)
	// a un array json {"timeStamp": value}
	//
	void timeSerieToJSON(const utils::timeSerie_t& ts, json& timeSerieJSON)
	{
		std::map<std::string, double> ts_modif ;
		for(const auto& [stime, value] : ts){
			ts_modif[std::to_string(stime)] = value;
		}
		
		timeSerieJSON = json(ts_modif);
	}
	
	//
	// transforma una serie de tiempo JSON (AKA array json {"timeStamp": value})
	// a una serie de tiempo  timeSerie_t(AKA std::map<uint32_t, double>) 
	//
	void JSONToTimeSerie(const json& timeSerieJSON, utils::timeSerie_t& ts)
	{
		ts.clear();
		
		for(const auto& reg : timeSerieJSON.items()){
			uint32_t timeStamp = std::stoi(reg.key());
			double   value     = reg.value();

			ts[timeStamp] = value;
		}
	}
	
	utils::timeSerie_t JSONToTimeSerie(const json& timeSerieJSON)
	{
		 utils::timeSerie_t ts;
		 JSONToTimeSerie(timeSerieJSON, ts);
		 return(ts);
	}
	
	
	//
	// Determina la distancia DTW y el desfase temporal entre dos series de tiempo
	//
	std::tuple<utils::dtwDistance_t, utils::timeMeasure_t> metricsDTW(const utils::timeSerie_t& timeSerie_Qry, const utils::timeSerie_t& timeSerie_Ref){
		utils::dtwDistance_t distanceDTW;
		utils::timeMeasure_t avgTimeShift;
	
	
		//
		// Sacar las estampas de tiempo y los datos de las serie de tiempo
		//
		utils::timestampSerie_t timeStamp_Qry;
		utils::dataSerie_t      data_Qry;
		utils::extractTimeValueFromTS(timeSerie_Qry, timeStamp_Qry, data_Qry);
	
	
		utils::timestampSerie_t timeStamp_Ref;
		utils::dataSerie_t      data_Ref;
		utils::extractTimeValueFromTS(timeSerie_Ref, timeStamp_Ref, data_Ref);
	
		dtw tsCompare;

		tsCompare.windowSize(0);	
		tsCompare.referenceSerie(data_Ref);						
		tsCompare.querySerie(data_Qry);	
		tsCompare.run();				

		distanceDTW  = tsCompare.DTWdistanceOE();
		avgTimeShift = tsCompare.DTWavgTimeShiftOE(timeStamp_Qry, timeStamp_Ref);
	
	
		return( std::make_tuple(distanceDTW, avgTimeShift) );
	}
	
	double determineMetric(utils::dtwDistance_t d, utils::timeMeasure_t t){
		double metric;
	
		t = std::abs(t);
	
		/*
		//
		// opción 1
		//
		metric = d * t;
		*/
	
		/*
		//
		// opción 2
		// Normalizar el desafase temporal [0, +inf] -> [0, 1)
		// 
		//
		double timeBias = 120;
	
		t /= timeBias;
		double nDTWdist = d / (1 + d);
		double nTimeShf = t / (1 + t);
		
		metric = ( nDTWdist + nTimeShf ) / 2;
		*/
	
	
		//
		// opción 3
		// Normalizar el desafase temporal [0, +inf] -> [0, 1)
		// Hacer lo mismo para la distancia DTW
		//
		double timeBias = 30; //120;
		//double dtwBias  = 0.3;
	
		t /= timeBias;
		//d /= dtwBias;
		//double nDTWdist = d / (1 + d);
		//double nTimeShf = t / (1 + t);
	
		d = d / (1 + d);
		t = t / (1 + t);
		
		double alpha = 0.5; //0.3;
		metric = d * alpha + t * (1 - alpha);
	
		return( metric);
	}
	
	
	utils::heatMapDistance_t heatMapDistance(heatMapFilePath_t hmFileA, heatMapFilePath_t hmFileB){
		utils::heatMapDistance_t heatMapDistance = 0.0;
		
		//Timer<std::chrono::microseconds> timer1, timer2;
		
		//timer1.start();
		PGM heatMapA(hmFileA, " ");
		PGM heatMapB(hmFileB, " ");

		try{
			heatMapA.load();
			heatMapB.load();
		} catch(std::exception& e){
			std::cerr << e.what() <<std::endl;
			exit(EXIT_FAILURE);
		}
		//timer1.stop();
		
		//timer2.start();
		heatMapDistance = PGM::similarity(heatMapA, heatMapB);
		//timer2.stop();


		//std::cout << "access file: " << timer1.elapsed() << ", distance time: " << timer2.elapsed() << std::endl;
		return(heatMapDistance);
	}
	
	void showProgress(const std::string& label, double progress)
	{	
		if( progress >= 100){
			progress = 100;
		}
		
		std::cout << "\r\x1B[2K" << std::fixed << std::setprecision(1);
		std::cout << label << progress << "%" << std::flush;
		
		if( progress >= 100){
			std::cout << "\r\x1B[2K" << std::flush;
		}
	}
	
}

namespace utils{
	ZoneBasic::ZoneBasic()
	{
		_projector = GeographicLib::LocalCartesian(-20.216667, -70.166667, 0, GeographicLib::Geocentric::WGS84());
	}
		
	ZoneBasic::ZoneBasic(json zoneGeoData, json zoneEvacTimeSerieJSON) : _zoneGeoData(zoneGeoData), _zoneEvacTimeSerieJSON(zoneEvacTimeSerieJSON)
	{
		_projector = GeographicLib::LocalCartesian(-20.216667, -70.166667, 0, GeographicLib::Geocentric::WGS84());
		
		double lon = _zoneGeoData["centroide"][0];
		double lat = _zoneGeoData["centroide"][1];
		
		double rw = _zoneGeoData["requiv"][0];
		double rh = _zoneGeoData["requiv"][1];
		
		this->setCentroidWGS84(lon, lat);
		this->setRequiv(rw, rh);
		
		_zoneEvacTimeSerie = utils::JSONToTimeSerie(zoneEvacTimeSerieJSON);
		
	}

	void ZoneBasic::setCentroidWGS84(double lon, double lat)
	{
		double x,y,z,h;
		h = 0.0;
		_projector.Forward(lat, lon, h, x, y, z);
	
		_centroidWSG84 = Point2D(lon,lat);
		_centroid      = Point2D(x,y);
	}

	void ZoneBasic::setRequiv(double rw, double rh)
	{
		_rEquiv = Vector2D(rw, rh);
	}

	Point2D ZoneBasic::getCentroidWGS84()
	{
		return(_centroidWSG84);
	}

	Vector2D ZoneBasic::getRequiv()
	{
		return(_rEquiv);
	}
	
	utils::timeSerie_t ZoneBasic::getTimeSerie()
	{
		return(_zoneEvacTimeSerie);
	}
	
	
	////////////////////////////////////////////////////////////////////////
	// zoneSimilarity implementation
	//
	void zonesSimilarity::sortZones(ZoneBasic& a, ZoneBasic& b)
	{
		ZoneBasic tmp;
	
		Vector2D zoneA_wh = a.getRequiv();
		Vector2D zoneB_wh = b.getRequiv();
	
		//if(a._rh > b._rh){
		if(zoneA_wh.y() > zoneB_wh.y()){
			tmp = a;
			a = b;
			b = tmp;
		}
	}


	double zonesSimilarity::jaccardIndex(ZoneBasic a, ZoneBasic b)
	{
		const GeographicLib::Geodesic& geod(GeographicLib::Geodesic::WGS84());
		double jaccard = 0.0;
	
		zonesSimilarity::sortZones(a, b);
	
	
		double distancia;
		Point2D centroidA = a.getCentroidWGS84();
		Point2D centroidB = b.getCentroidWGS84();
	
		geod.Inverse(centroidA.y(), centroidA.x(), centroidB.y(), centroidB.x(), distancia);
	
		Vector2D zA_wh = a.getRequiv();
		Vector2D zB_wh = b.getRequiv();
	
		//if(distancia <= abs(b._rw - a._rw)){
		if( distancia <= abs(zB_wh.x() - zA_wh.x()) ){
		
			//if(a._rw <= b._rw){//Caso 1
			if(zA_wh.x() <= zB_wh.x()){//Caso 1
				//jaccard = (a._rw*a._rh)/(b._rw*b._rh);
				jaccard = (zA_wh.x()*zA_wh.y())/(zB_wh.x()*zB_wh.y());
			}
			else{ //Caso 2
				//jaccard = (b._rw*a._rh)/(a._rw*a._rh+b._rw*b._rh-b._rw*a._rh);
				jaccard = (zB_wh.x()*zA_wh.y()) / (zA_wh.x()*zA_wh.y() + zB_wh.x()*zB_wh.y() - zB_wh.x()*zA_wh.y());
			}
		}
		else if(distancia > abs(zB_wh.x() - zA_wh.x())  && distancia <= (zB_wh.x() + zA_wh.x()) ){
			//double n = 2*a._rh*(a._rw+b._rw-distancia);
			double n = 2*zA_wh.y()*(zA_wh.x() + zB_wh.x() - distancia);
			//jaccard = n/(4*(a._rw*a._rh+b._rw*b._rh) - n);
			jaccard = n/(4*(zA_wh.x()*zA_wh.y() + zB_wh.x()*zB_wh.y()) - n);
		}
		else{
			jaccard = 0;
		}

		return(jaccard);
	}
	

} 









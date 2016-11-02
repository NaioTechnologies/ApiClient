//
//#ifndef IMAGE_MANAGER_HPP
//#define IMAGE_MANAGER_HPP
//
//class ImageManager
//{
//public:
//	ImageManager( );
//	~ImageManager( );
//
//	void start_image_server( );
//
//private:
//	void simulator_read_thread_function( );
//	void simulator_write_thread_function( );
//
//	void ozcore_server_thread_function( );
//	void ozcore_read_thread_function( );
//	void ozcore_write_thread_function( );
//
//private:
//	bool simulator_read_thread_started_;
//	std::thread simulator_read_thread_;
//
//	bool simulator_write_thread_started_;
//	std::thread simulator_write_thread_;
//
//	bool ozcore_server_thread_started_;
//	std::thread ozcore_server_thread_;
//	int ozcore_server_socket_;
//	int ozcore_client_socket_;
//	bool ozcore_client_connected_;
//
//};
//
//#endif //IMAGE_MANAGER_HPP
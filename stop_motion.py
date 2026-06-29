from Rosmaster_Lib import Rosmaster

bot = Rosmaster()
bot.clear_auto_report_data()
bot.create_receive_threading()

bot.set_car_motion(0, 0, 0)